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

namespace
{
    const INSTTYPE ZU = INSTTYPE_ZEROREG_UIMM;
    const INSTTYPE ZS = INSTTYPE_ZEROREG_SIMM;
    const INSTTYPE OU = INSTTYPE_ONEREG_UIMM;
    const INSTTYPE OS = INSTTYPE_ONEREG_SIMM;
    const INSTTYPE TU = INSTTYPE_TWOREG_UIMM;
    const INSTTYPE TS = INSTTYPE_TWOREG_SIMM;
}
INSTTYPE STRAIGHT64Decoder::s_opCodeToRegType[64]{
    ZU, ZU, ZU, ZS, OU, ZS, OU, OS,
    OS, TU, OS, TU, OS, TU, OS, TU,
    OS, TU, OU, TU, OS, TU, OU, TU,
    OS, TU, OU, OU, OU, TU, TU, TU,
    TU, TU, TU, OU, TU, OU, TU, OU,
    TU, OU, TU, OU, TU, OU, ZU, OS,
    ZU, OU, OS, OS, OS, OS, OS, TS,
    TS, TS, OU, OU, OU, OU,
};

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

    const u32 opcode = (codeWord >> 26) & 0x3f;
    const INSTTYPE instType = s_opCodeToRegType[opcode];
    
    out->Reg[0] = 0;
    switch (instType) {
    case INSTTYPE_ZEROREG_UIMM:
        out->Imm[0] = ExtractBits(codeWord, 0, 26);
        break;
    case INSTTYPE_ZEROREG_SIMM:
        out->Imm[0] = ExtractBits(static_cast<u64>(codeWord), 0, 26, true);
        break;
    case INSTTYPE_ONEREG_UIMM:
        out->Imm[0] = ExtractBits(codeWord, 0, 16);
        out->Reg[1] = ExtractBits(codeWord, 16, 10);
        break;
    case INSTTYPE_ONEREG_SIMM:
        out->Imm[0] = ExtractBits(static_cast<u64>(codeWord), 0, 16, true);
        out->Reg[1] = ExtractBits(codeWord, 16, 10);
        break;
    case INSTTYPE_TWOREG_UIMM:
        out->Imm[0] = ExtractBits(codeWord, 0, 6);
        out->Reg[1] = ExtractBits(codeWord, 16, 10);
        out->Reg[2] = ExtractBits(codeWord, 6, 10);
        break;
    case INSTTYPE_TWOREG_SIMM:
        out->Imm[0] = ExtractBits(codeWord, 0, 6);
        out->Reg[1] = ExtractBits(codeWord, 16, 10);
        out->Reg[2] = ExtractBits(codeWord, 6, 10);
        break;
    default:
        break;
    }
}

