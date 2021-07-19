#ifndef SRC_SHA256IDGENERATOR_HPP_
#define SRC_SHA256IDGENERATOR_HPP_

#include <iostream>
#include <string>

#include "immutable/idGenerator.hpp"
#include "immutable/pageId.hpp"

class Sha256IdGenerator : public IdGenerator {
public:
    virtual PageId generateId(std::string const& content) const
    {
        FILE* file = popen(("printf -- \'" + content + "\' | sha256sum").c_str(), "r");
        char buffer[64];
        if (fscanf(file, "%64s", buffer) != 1)
            exit(1);
        pclose(file);

        return PageId(buffer);
    }
};

#endif /* SRC_SHA256IDGENERATOR_HPP_ */
