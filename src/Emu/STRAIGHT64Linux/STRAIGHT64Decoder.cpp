// 
// Copyright (c) 2005-2008 Kenichi Watanabe.
// Copyright (c) 2005-2008 Yasuhiro Watari.
// Copyright (c) 2005-2008 Hironori Ichibayashi.
// Copyright (c) 2008-2009 Kazuo Horio.
// Copyright (c) 2009-2015 Naruki Kurata.
// Copyright (c) 2005-2015 Masahiro Goshima.
// Copyright (c) 2005-2017 Ryota Shioya.
// 
// This software is provided 'as-is', without any express or implied
// warranty. In no event will the authors be held liable for any damages
// arising from the use of this software.
// 
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 
// 1. The origin of this software must not be misrepresented; you must not
// claim that you wrote the original software. If you use this software
// in a product, an acknowledgment in the product documentation would be
// appreciated but is not required.
// 
// 2. Altered source versions must be plainly marked as such, and must not be
// misrepresented as being the original software.
// 
// 3. This notice may not be removed or altered from any source
// distribution.
// 
// 


#include <pch.h>

#include "Emu/STRAIGHT64Linux/STRAIGHT64Decoder.h"

#include "SysDeps/Endian.h"
#include "Emu/Utility/DecoderUtility.h"


using namespace std;
using namespace boost;
using namespace Onikiri;
using namespace Onikiri::EmulatorUtility;
using namespace Onikiri::STRAIGHT64Linux;

const u32 OPCODE_ST8 = 0b000111;
const u32 OPCODE_ST16 = 0b100111;
const u32 OPCODE_ST32 = 0b010111;
const u32 OPCODE_ST64 = 0b110111;
const u32 OPCODE_BLT = 0b000011;
const u32 OPCODE_BGE = 0b100011;
const u32 OPCODE_BLTU = 0b010011;
const u32 OPCODE_BGEU = 0b110011;
const u32 OPCODE_BEQ = 0b001011;
const u32 OPCODE_BNE = 0b101011;

STRAIGHT64Decoder::DecodedInsn::DecodedInsn()
{
    clear();
}

void STRAIGHT64Decoder::DecodedInsn::clear()
{
    CodeWord = 0;

    std::fill(Imm.begin(), Imm.end(), 0);
    std::fill(Reg.begin(), Reg.end(), -1);
}

void STRAIGHT64Decoder::Decode(u32 codeWord, DecodedInsn* out)
{
    out->clear();
    out->CodeWord = codeWord;
    out->instType = GetInstType(codeWord);
    out->Reg[0] = 0;

    switch (out->instType){
    case INSTTYPE_STB:
        out->Imm[0] = ExtractBits(codeWord, 7, 12, true);
        out->Reg[0] = ExtractBits(codeWord, 26, 7, true);
        out->Reg[1] = ExtractBits(codeWord, 19, 7, true);
        break;
    default:
        THROW_RUNTIME_ERROR("decode error:");
    }


    for (unsigned int i = 1; i < out->Reg.size(); i++)
    {
        if (out->Reg[i] == 0)
        {
            out->Reg[i] = STRAIGHT64Info::ZeroRegIndex;
        }
    }
}

INSTTYPE STRAIGHT64Decoder::GetInstType(u32 codeWord)
{
    if ((codeWord & 0x1f) != 0b11011) return INSTTYPE_STB;

    THROW_RUNTIME_ERROR("decode error: %d = %b", codeWord, codeWord);
}
