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


#ifndef EMU_STRAIGHT64LINUX_STRAIGHT64_OPINFO_H
#define EMU_STRAIGHT64LINUX_STRAIGHT64_OPINFO_H

#include "Emu/Utility/CommonOpInfo.h"
#include "Emu/STRAIGHT64Linux/STRAIGHT64Info.h"

namespace Onikiri {
    namespace STRAIGHT64Linux {

        class STRAIGHT64OpInfo : public EmulatorUtility::CommonOpInfo<STRAIGHT64Info>
        {
        public:
            explicit STRAIGHT64OpInfo(OpClass opClass) : CommonOpInfo<STRAIGHT64Info>(opClass) {}
            
            // todo: rewrite
            bool isSPADDi() const { return strcmp(m_mnemonic, "SPADDi") == 0; }
            bool isRPINC() const { return strcmp(m_mnemonic, "NOP/RPINC") == 0; }
            bool isSPLDSTorAUiSP() const { return strstr(m_mnemonic, "SPLD") != nullptr || strstr(m_mnemonic, "SPST") != nullptr || strcmp(m_mnemonic, "AUiSP") == 0; }
            bool isSPST() const { return strstr(m_mnemonic, "SPST") != nullptr; }

            bool isSyscall() const { return m_opClass.GetCode() == OpClassCode::OpClassCode::syscall; }
            bool isSyscallBranch() const { return m_opClass.GetCode() == OpClassCode::OpClassCode::syscall_branch; }
        };

    } // namespace STRAIGHT64Linux
} // namespace Onikiri

#endif

