// 
// Copyright (c) 2005-2008 Kenichi Watanabe.
// Copyright (c) 2005-2008 Yasuhiro Watari.
// Copyright (c) 2005-2008 Hironori Ichibayashi.
// Copyright (c) 2008-2009 Kazuo Horio.
// Copyright (c) 2009-2015 Naruki Kurata.
// Copyright (c) 2005-2015 Ryota Shioya.
// Copyright (c) 2005-2015 Masahiro Goshima.
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


#include "pch.h"

#include "Sim/Pipeline/Scheduler/IssueSelector/InorderIssueSelector.h"
#include "Sim/Op/Op.h"

#include <algorithm>

using namespace Onikiri;

InorderIssueSelector::InorderIssueSelector()
{

}

InorderIssueSelector::~InorderIssueSelector()
{

}

void InorderIssueSelector::Initialize( InitPhase phase )
{
}

void InorderIssueSelector::Finalize()
{
    ReleaseParam();
}

void InorderIssueSelector::EvaluateSelect( Scheduler* scheduler )
{
    // Select issued ops.
    int issueCount = 0;
    int issueWidth = scheduler->GetIssueWidth();

    const OpList& notReadyOps = scheduler->GetNotReadyOps();
    u64 minNotReadyID = UINT64_MAX;
    for( auto op : notReadyOps ) {
        minNotReadyID = std::min( minNotReadyID, op->GetGlobalSerialID() );
    }

    const OpList& readyOps = scheduler->GetReadyOps();
    const SchedulingOps& wokeUpOps = scheduler->GetWokeUpOps();

    pool_vector< OpIterator > readyAndWokeUpOps;
    for( auto op : readyOps ) {
        readyAndWokeUpOps.push_back( op );
    }
    for( auto op : wokeUpOps ) {
        readyAndWokeUpOps.push_back( op );
    }

    std::sort( readyAndWokeUpOps.begin(), readyAndWokeUpOps.end(),
        []( OpIterator lhs, OpIterator rhs ) { return lhs->GetGlobalSerialID() < rhs->GetGlobalSerialID(); }
    );

    for( auto op : readyAndWokeUpOps ) {
        // If there are any older ops, the op is not selected.
        if( notReadyOps.size() > 0 &&
            minNotReadyID < op->GetGlobalSerialID()
        ){
            break;
        }

        if( scheduler->CanSelect( op ) ) {
            scheduler->ReserveSelect( op );
            ++issueCount;
            if( issueCount >= issueWidth ) {
                break;
            }
        }
        else{
            g_dumper.Dump( DS_WAITING_UNIT, op );
        }
    }
}
