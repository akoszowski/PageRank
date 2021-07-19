#ifndef SRC_MULTITHREADEDPAGERANKCOMPUTER_HPP_
#define SRC_MULTITHREADEDPAGERANKCOMPUTER_HPP_

#include <atomic>
#include <condition_variable>
#include <mutex>

#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "immutable/network.hpp"
#include "immutable/pageIdAndRank.hpp"
#include "immutable/pageRankComputer.hpp"

class MultiThreadedPageRankComputer : public PageRankComputer {
public:
    MultiThreadedPageRankComputer(uint32_t numThreadsArg)
        : numThreads(numThreadsArg) {};

    std::vector<PageIdAndRank> computeForNetwork(
        Network const& network, double alpha, uint32_t iterations, double tolerance) const
    {
        // Clearing static members.
        initStructs();

        // Generating ids.
        std::atomic<size_t> curPage(0);
        std::vector<std::thread> vecOfThreads;
        for (uint32_t i = 0; i < numThreads; ++i) {
            vecOfThreads.push_back(std::thread(
                &MultiThreadedPageRankComputer::generateWork, std::ref(curPage), std::ref(network)));
        }

        for (uint32_t i = 0; i < numThreads; ++i) {
            vecOfThreads[i].join();
        }
        vecOfThreads.clear();

        // Initializing default values for data structures.
        // Building (graph) representation of network.
        for (auto const& page : network.getPages()) {
            pageHashMap[page.getId()] = 1.0 / network.getSize();

            numLinks[page.getId()] = page.getLinks().size();

            if (page.getLinks().size() == 0) {
                danglingNodes.insert(page.getId());
            }

            for (auto const& link : page.getLinks()) {
                edges[link].push_back(page.getId());
            }
        }

        // Determining pageRank values for each page in network.
        for (uint32_t i = 0; i < iterations; ++i) {
            previousPageHashMap = pageHashMap;

            double dangleSum = 0;
            for (auto const& danglingNode : danglingNodes) {
                dangleSum += previousPageHashMap[danglingNode];
            }
            dangleSum = dangleSum * alpha;

            double difference = 0;
            size_t elemsPerThread = pageHashMap.size() / numThreads;
            std::unordered_map<PageId, PageRank, PageIdHash>::iterator mapBeg, mapEnd = pageHashMap.begin();
            for (uint32_t j = 0; j < numThreads; ++j) {
                mapBeg = mapEnd;
                mapEnd = (j == numThreads - 1 ? pageHashMap.end() : std::next(mapBeg, elemsPerThread));
                vecOfThreads.push_back(
                    std::thread(&MultiThreadedPageRankComputer::iterationWork, std::ref(difference),
                        dangleSum, alpha, std::ref(network), mapBeg, mapEnd));
            }
            for (uint32_t j = 0; j < numThreads; ++j) {
                vecOfThreads[j].join();
            }
            vecOfThreads.clear();

            if (difference < tolerance) {
                std::vector<PageIdAndRank> result;
                for (auto& iter : pageHashMap) {
                    result.push_back(PageIdAndRank(iter.first, iter.second));
                }

                ASSERT(result.size() == network.getSize(),
                    "Invalid result size=" << result.size() << ", for network" << network);

                return result;
            }
        }

        ASSERT(false, "Not able to find result in iterations=" << iterations);
    }

    std::string getName() const
    {
        return "MultiThreadedPageRankComputer[" + std::to_string(this->numThreads) + "]";
    }

private:
    uint32_t numThreads;

    static std::mutex mutex;
    static std::unordered_map<PageId, PageRank, PageIdHash> pageHashMap;
    static std::unordered_map<PageId, PageRank, PageIdHash> previousPageHashMap;
    static std::unordered_map<PageId, uint32_t, PageIdHash> numLinks;
    static std::unordered_set<PageId, PageIdHash> danglingNodes;
    static std::unordered_map<PageId, std::vector<PageId>, PageIdHash> edges;

    void initStructs() const
    {
        pageHashMap.clear();
        numLinks.clear();
        danglingNodes.clear();
        edges.clear();
    }

    // Using std::atomic here, since generatingId time basing on the content
    // of the page may considerably differ for each page.
    static void generateWork(std::atomic<size_t>& curPage, Network const& network)
    {
        size_t cur;
        while ((cur = curPage++) < network.getPages().size()) {
            Page const& myPage = network.getPages()[cur];
            myPage.generateId(network.getGenerator());
        }
    }

    // Each thread determines values for pages that are in range of [beg, end)
    // iterators. First method, with std::atomic (used in generateWork function)
    // is especially ineffective for networks with low number of links, due to
    // mutex addons.
    static void iterationWork(double& difference, double dangleSum, double alpha,
        Network const& network, std::unordered_map<PageId, PageRank, PageIdHash>::iterator beg,
        std::unordered_map<PageId, PageRank, PageIdHash>::iterator end)
    {
        double curDif = 0;
        for (auto it = beg; it != end; ++it) {
            PageId myPageId = it->first;

            double danglingWeight = 1.0 / network.getSize();
            it->second = dangleSum * danglingWeight + (1.0 - alpha) / network.getSize();

            if (edges.count(myPageId) > 0) {
                for (auto const& link : edges[myPageId]) {
                    it->second += alpha * previousPageHashMap[link] / numLinks[link];
                }
            }

            curDif += std::abs(previousPageHashMap[myPageId] - pageHashMap[myPageId]);
        }

        {
            std::lock_guard<std::mutex> lck(mutex);
            difference += curDif;
        }
    }
};

std::mutex MultiThreadedPageRankComputer::mutex;
std::unordered_map<PageId, PageRank, PageIdHash> MultiThreadedPageRankComputer::pageHashMap;
std::unordered_map<PageId, PageRank, PageIdHash> MultiThreadedPageRankComputer::previousPageHashMap;
std::unordered_map<PageId, uint32_t, PageIdHash> MultiThreadedPageRankComputer::numLinks;
std::unordered_set<PageId, PageIdHash> MultiThreadedPageRankComputer::danglingNodes;
std::unordered_map<PageId, std::vector<PageId>, PageIdHash> MultiThreadedPageRankComputer::edges;

#endif /* SRC_MULTITHREADEDPAGERANKCOMPUTER_HPP_ */
