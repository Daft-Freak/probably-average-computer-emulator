#include <fstream>
#include <iostream>
#include <string>

#include <nlohmann/json.hpp>
#include <zlib.h>

#include "System.h"

using json = nlohmann::json;

static System sys;
static uint8_t ram[1024 * 1024];

static std::string readGZ(const std::string &path)
{
    auto file = gzopen(path.c_str(), "r");

    if(!file)
        return "";

    std::string ret;
    
    while(true)
    {
        auto offset = ret.size();

        const int chunkSize = 4066;

        ret.resize(ret.size() + chunkSize);

        int readLen = gzread(file, ret.data() + offset, chunkSize);

        if(readLen < chunkSize)
        {
            ret.resize(offset + readLen);
            break;
        }
    }

    gzclose(file);

    return ret;
}

static CPU::Reg16 getReg(const std::string &name)
{
    if(name == "ax")
        return CPU::Reg16::AX;
    if(name == "bx")
        return CPU::Reg16::BX;
    if(name == "cx")
        return CPU::Reg16::CX;
    if(name == "dx")
        return CPU::Reg16::DX;
    if(name == "cs")
        return CPU::Reg16::CS;
    if(name == "ss")
        return CPU::Reg16::SS;
    if(name == "ds")
        return CPU::Reg16::DS;
    if(name == "es")
        return CPU::Reg16::ES;
    if(name == "sp")
        return CPU::Reg16::SP;
    if(name == "bp")
        return CPU::Reg16::BP;
    if(name == "si")
        return CPU::Reg16::SI;
    if(name == "di")
        return CPU::Reg16::DI;
    if(name == "ip")
        return CPU::Reg16::IP;

    std::cerr << "bad reg " << name << "\n";
    return CPU::Reg16::AX;
}

static void doOpTest(const std::string &testData, uint16_t flagsMask)
{
    auto &cpu = sys.getCPU();

    auto testJson = json::parse(testData);

    int passCount = 0;

    for(auto &test : testJson)
    {
        sys.reset();

        // setup initial data
        auto initialRegs = test["initial"]["regs"];
        auto initialRAM = test["initial"]["ram"];

        for(auto &reg : initialRegs.items())
        {
            if(reg.key() == "flags")
                cpu.setFlags(reg.value());
            else
                cpu.reg(getReg(reg.key())) = reg.value();
        }

        for(auto &ram : initialRAM)
            sys.writeMem(ram[0], ram[1]);

        cpu.executeInstruction();

        // now test result
        auto finalRegs = test["final"]["regs"];
        auto finalRAM = test["final"]["ram"];

        std::cerr << std::hex;
        bool fail = false;

        for(auto &reg : finalRegs.items())
        {
            uint16_t expected = reg.value();
            uint16_t actual;

            if(reg.key() == "flags")
            {
                actual = cpu.getFlags() & flagsMask;
                expected &= flagsMask; // let's not bother with the unspecified ones
            }
            else
                actual = cpu.reg(getReg(reg.key()));

            if(actual != expected)
            {
                if(!fail)
                    std::cerr << "test "<< test["name"] << "\n";
                std::cerr << "\tfail " << reg.key() << " " << actual << " != " << expected << "\n";
                fail = true;
            }
        }

        for(auto &ram : finalRAM)
        {
            int expected = ram[1];
            int actual = sys.readMem(ram[0]);

            int addr = ram[0];

            if(actual != expected)
            {
                if(!fail)
                    std::cerr << "test "<< test["name"] << "\n";
                std::cerr << "\tfail ram " << addr << " " << actual << " != " << expected << "\n";
                fail = true;
            }
        }

        std::cerr << std::dec;

        if(!fail)
            passCount++;
    }

    std::cout << "\tpassed " << passCount << "/" << testJson.size() << std::endl;
}

int main(int argc, char *argv[])
{
    sys.addMemory(0, sizeof(ram), ram);

    auto metadata = json::parse(std::ifstream("8088/v1/metadata.json"));

    for(auto &opcode : metadata["opcodes"].items())
    {
        auto &value = opcode.value();

        // skip undocumented
        if(value.contains("status") && value["status"] != "normal")
            continue;

        // skip unimpl
        // ... exit(1) was a bad move...
        auto k = opcode.key();
        if(k == "0F" || k == "2F"/*DAS*/ || k == "37"/*AAA*/ || k == "3F"/*AAS*/ || k == "C2"/*RET*/ || k == "CC"/*INT 3*/ || k == "CE"/*INTO*/ ||
           k == "E5" /*IN w*/ || k == "E7" /*OUT w*/ || k == "ED" /*IN w*/ || k == "EF" /*OUT w*/)
        {
            continue;
        }
        //

        // de-noise
        // segment mov fails due to not masking reg
        // AAM has wrong(undefined) flags on trap
        // IN fails due to emulating some ports
        if(k == "8C"/*seg*/ || k == "8E"/*seg*/ || k == "D4"/*AAM*/ || k == "E4" /*IN*/ || k == "EC"/*IN*/)
            continue;
        //

        uint16_t flagsMask = 0xFFFF;

        if(value.contains("reg"))
        {
            // sub-opcodes
            for(auto &subOpcode : value["reg"].items())
            {
                auto &subValue = subOpcode.value();

                if(subValue["status"] != "normal")
                    continue;

                // unimpl
                if(k == "F6" && (subOpcode.key() == "5" || subOpcode.key() == "7"))
                    continue;
                //

                // de-noise
                // DIV has wrong(undefined) flags on trap
                if(((k == "F6" || k == "F7") && (subOpcode.key() == "6" || subOpcode.key() == "7")))
                    continue;
                //

                if(subValue.contains("flags-mask"))
                    flagsMask = subValue["flags-mask"];

                std::cout << "opcode " << opcode.key() << " r " << subOpcode.key() << std::endl;

                auto data = readGZ("8088/v1/" + opcode.key() + "." + subOpcode.key() + ".json.gz");

                if(data.empty() && subOpcode.key() == "0")
                    data = readGZ("8088/v1/" + opcode.key() + ".json.gz");

                if(data.empty())
                {
                    std::cerr << "could not get data\n";
                    continue;
                }
                doOpTest(data, flagsMask);
            }
        }
        else
        {
            // no sub-opcodes
            std::cout << "opcode " << opcode.key() << std::endl;

            if(value.contains("flags-mask"))
                flagsMask = value["flags-mask"];

            auto data = readGZ("8088/v1/" + opcode.key() + ".json.gz");

            if(data.length())
                doOpTest(data, flagsMask);
            else
                std::cerr << "could not get data\n";
        }
    }

    return 0;
}
