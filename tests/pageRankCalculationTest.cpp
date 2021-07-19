#include <iostream>
#include <vector>

#include "../src/immutable/common.hpp"
#include "../src/immutable/pageIdAndRank.hpp"
#include "../src/multiThreadedPageRankComputer.hpp"
#include "../src/singleThreadedPageRankComputer.hpp"

#include "./lib/networkGenerator.hpp"
#include "./lib/resultVerificator.hpp"
#include "./lib/simpleIdGenerator.hpp"

struct TestScenario {
    uint32_t numberOfNodes;
    double alpha;
    uint32_t iterations;
    double tolerance;

    std::vector<PageRank> expectedResult;
};

int main()
{
    std::vector<TestScenario> scenarios = {
        { 3, 0.85, 100, 0.0000001, { 0.25974019022001016, 0.25974019022001016, 0.48051961955997957 } },
        { 3, 0.15, 100, 0.0000001, { 0.31746031249999995, 0.31746031249999995, 0.36507937499999993 } },
        { 5, 0.85, 100, 0.0000001, { 0.030000000000000006, 0.030000000000000006, 0.3133333333333334, 0.4067843162091876, 0.21988235045747923 } },
        { 100, 0.85, 100, 0.0000001, { 0.015216898591602811, 0.011154953239472692, 0.008919340472247617, 0.006515061161406849, 0.004896419452867842, 0.0032465927984213164, 0.020224833579686366, 0.010382740826417693, 0.006525752304337512, 0.003583531363201988, 0.020197807361602767, 0.008833041571619593, 0.004946563116221868, 0.028679664324516234, 0.008919340472247617, 0.0036170041022833485, 0.015608850267547129, 0.006022231054928431, 0.028907507795329364, 0.006939811140326575, 0.0021308146152911782, 0.0070585378786100165, 0.0016107246819770775, 0.00650802417735698, 0.02003861265823865, 0.004933353190124864, 0.010976421404941317, 0.002863830255008852, 0.006520872324044255, 0.01469670347578044, 0.0032151800629584963, 0.006425308575660245, 0.010433863294813082, 0.02674802794443123, 0.0036170041022833485, 0.006370612217008803, 0.008840244399683547, 0.014994090512809288, 0.029003335311711634, 0.0028638302550088516, 0.0036170041022833485, 0.004989264144574225, 0.006030289150361447, 0.006581780627663144, 0.006862043252136317, 0.006970113326425838, 0.0070585378786100165, 0.006872666058812157, 0.006581780627663144, 0.0060986317003086325, 0.004989264144574224, 0.003617004102283349, 0.0028638302550088516, 0.02674802794443123, 0.015196391329455083, 0.008807603436295476, 0.006425308575660245, 0.003617004102283349, 0.028311642977640016, 0.01065017730961612, 0.006360175216508896, 0.003246592798421317, 0.015026095944588546, 0.006581780627663144, 0.0028638302550088516, 0.01115495323947269, 0.004989264144574225, 0.02025614368986428, 0.006515061161406848, 0.0016107246819770775, 0.006986986193119979, 0.0021308146152911782, 0.006939811140326574, 0.029014640985893042, 0.006022231054928429, 0.015464402170330342, 0.0036170041022833494, 0.008767257088578435, 0.02788022286649839, 0.004891130729950755, 0.008919340472247617, 0.020224833579686366, 0.0036170041022833494, 0.006581780627663144, 0.010516753985886161, 0.019781476726107097, 0.003246592798421317, 0.004896419452867841, 0.006581780627663144, 0.008819388486102548, 0.01103046678091042, 0.015216898591602813, 0.02027266166339678, 0.028311642977640016, 0.0016107246819770775, 0.0021308146152911782, 0.0021308146152911782, 0.0016107246819770775, 0.027880222866498397, 0.020224833579686366 } }
    };
    std::vector<std::shared_ptr<PageRankComputer>> computersToTest = {
        std::shared_ptr<PageRankComputer>(new SingleThreadedPageRankComputer {}),
        std::shared_ptr<PageRankComputer>(new MultiThreadedPageRankComputer { 1 }),
        std::shared_ptr<PageRankComputer>(new MultiThreadedPageRankComputer { 2 }),
        std::shared_ptr<PageRankComputer>(new MultiThreadedPageRankComputer { 3 }),
        std::shared_ptr<PageRankComputer>(new MultiThreadedPageRankComputer { 4 }),
        std::shared_ptr<PageRankComputer>(new MultiThreadedPageRankComputer { 5 }),
        std::shared_ptr<PageRankComputer>(new MultiThreadedPageRankComputer { 7 }),
        std::shared_ptr<PageRankComputer>(new MultiThreadedPageRankComputer { 8 }),
        std::shared_ptr<PageRankComputer>(new MultiThreadedPageRankComputer { 9 }),
    };

    /*std::vector<std::shared_ptr<PageRankComputer>> computersToTest = {
        //std::shared_ptr<PageRankComputer>(new SingleThreadedPageRankComputer {}),
        std::shared_ptr<PageRankComputer>(new MultiThreadedPageRankComputer { 1 }),
        std::shared_ptr<PageRankComputer>(new MultiThreadedPageRankComputer { 2 }),
        std::shared_ptr<PageRankComputer>(new MultiThreadedPageRankComputer { 3 }),
        std::shared_ptr<PageRankComputer>(new MultiThreadedPageRankComputer { 4 })
    };*/

    SimpleIdGenerator idGenerator("b7628d82a284526971095162ba34be8bc05c6e06b9face83b46c2813f7f2157b");
    SimpleNetworkGenerator networkGenerator(idGenerator);
    for (auto computer : computersToTest) {

        for (auto scenario : scenarios) {
            std::cout << "Starting scenario with numberOfNodes=" << scenario.numberOfNodes << ", alpha=" << scenario.alpha << std::endl;
            auto result = computer->computeForNetwork(
                networkGenerator.generateNetworkOfSize(scenario.numberOfNodes),
                scenario.alpha,
                scenario.iterations,
                scenario.tolerance);
            ResultVerificator::verifyResults(result, scenario.expectedResult, networkGenerator);
            // std::cout << "Scenario finished with successed" << std::endl;
        }
    }

    return 0;
}
