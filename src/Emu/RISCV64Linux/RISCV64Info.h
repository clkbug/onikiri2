// 
// Copyright (c) 2005-2008 Kenichi Watanabe.
// Copyright (c) 2005-2008 Yasuhiro Watari.
// Copyright (c) 2005-2008 Hironori Ichibayashi.
// Copyright (c) 2008-2009 Kazuo Horio.
// Copyright (c) 2009-2015 Naruki Kurata.
// Copyright (c) 2005-2015 Masahiro Goshima.
// Copyright (c) 2005-2018 Ryota Shioya.
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


#ifndef EMU_RISCV64LINUX_RISCV64_INFO_H
#define EMU_RISCV64LINUX_RISCV64_INFO_H

#include "Interface/ISAInfo.h"

namespace Onikiri {
    namespace RISCV64Linux {

        // RISCV64LinuxのISA情報
        class RISCV64Info : public ISAInfoIF
        {
        public:
            static const int InstructionWordBitSize = 32;
            static const int MaxSrcRegCount = 3;
            static const int MaxDstRegCount = 2;
            static const int MaxImmCount = 2;
            // Int : 32
            // FP  : 32
            // dummy0 : 1 (Alpha とセグメント数をあわせるため)
            // dummy1 : 1
            static const int RegisterCount = 32+32+1+1;
            //static const int REG_CSR     = 66;

            static const int MAX_MEMORY_ACCESS_WIDTH = 8;

            virtual ISA_TYPE GetISAType();
            virtual int GetInstructionWordBitSize();
            virtual int GetRegisterWordBitSize();
            virtual int GetRegisterCount();
            virtual int GetAddressSpaceBitSize();
            virtual int GetMaxSrcRegCount();
            virtual int GetMaxDstRegCount();
            virtual int GetRegisterSegmentID(int regNum);
            virtual int GetRegisterSegmentCount();
            virtual int GetMaxOpInfoCountPerPC();
            virtual int GetMaxMemoryAccessByteSize();
            virtual bool IsLittleEndian();
        };


    } // namespace RISCV64Linux
} // namespace Onikiri

#endif
