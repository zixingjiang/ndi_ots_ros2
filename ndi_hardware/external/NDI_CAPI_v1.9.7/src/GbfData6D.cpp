//----------------------------------------------------------------------------
//
//  Copyright (C) 2017, Northern Digital Inc. All rights reserved.
//
//  All Northern Digital Inc. ("NDI") Media and/or Sample Code and/or Sample Code
//  Documentation (collectively referred to as "Sample Code") is licensed and provided "as
//  is" without warranty of any kind. The licensee, by use of the Sample Code, warrants to
//  NDI that the Sample Code is fit for the use and purpose for which the licensee intends to
//  use the Sample Code. NDI makes no warranties, express or implied, that the functions
//  contained in the Sample Code will meet the licensee's requirements or that the operation
//  of the programs contained therein will be error free. This warranty as expressed herein is
//  exclusive and NDI expressly disclaims any and all express and/or implied, in fact or in
//  law, warranties, representations, and conditions of every kind pertaining in any way to
//  the Sample Code licensed and provided by NDI hereunder, including without limitation,
//  each warranty and/or condition of quality, merchantability, description, operation,
//  adequacy, suitability, fitness for particular purpose, title, interference with use or
//  enjoyment, and/or non infringement, whether express or implied by statute, common law,
//  usage of trade, course of dealing, custom, or otherwise. No NDI dealer, distributor, agent
//  or employee is authorized to make any modification or addition to this warranty.
//  In no event shall NDI nor any of its employees be liable for any direct, indirect,
//  incidental, special, exemplary, or consequential damages, sundry damages or any
//  damages whatsoever, including, but not limited to, procurement of substitute goods or
//  services, loss of use, data or profits, or business interruption, however caused. In no
//  event shall NDI's liability to the licensee exceed the amount paid by the licensee for the
//  Sample Code or any NDI products that accompany the Sample Code. The said limitations
//  and exclusions of liability shall apply whether or not any such damages are construed as
//  arising from a breach of a representation, warranty, guarantee, covenant, obligation,
//  condition or fundamental term or on any theory of liability, whether in contract, strict
//  liability, or tort (including negligence or otherwise) arising in any way out of the use of
//  the Sample Code even if advised of the possibility of such damage. In no event shall
//  NDI be liable for any claims, losses, damages, judgments, costs, awards, expenses or
//  liabilities of any kind whatsoever arising directly or indirectly from any injury to person
//  or property, arising from the Sample Code or any use thereof
//
//----------------------------------------------------------------------------

#include <iomanip>
#include <sstream>

#include "GbfData6D.h"

GbfData6D::GbfData6D(BufferedReader& reader, int numberOfTools)
{
    for ( int i = 0; i < numberOfTools; i++)
    {
        Transform data6D;
        data6D.toolHandle = reader.get_uint16();
        data6D.status = reader.get_uint16();

        if (data6D.isMissing())
        {
            data6D.q0 = BAD_FLOAT;
            data6D.qx = BAD_FLOAT;
            data6D.qy = BAD_FLOAT;
            data6D.qz = BAD_FLOAT;
            data6D.tx = BAD_FLOAT;
            data6D.ty = BAD_FLOAT;
            data6D.tz = BAD_FLOAT;
            data6D.error = BAD_FLOAT;
        }
        else
        {
            data6D.q0 = reader.get_double();
            data6D.qx = reader.get_double();
            data6D.qy = reader.get_double();
            data6D.qz = reader.get_double();
            data6D.tx = reader.get_double();
            data6D.ty = reader.get_double();
            data6D.tz = reader.get_double();
            data6D.error = reader.get_double();
        }
        toolTransforms.push_back(data6D);
    }
}

std::string GbfData6D::toString() const
{
    std::stringstream stream;
    stream << std::hex << std::setfill('0') << std::setprecision(4);
    stream << "-----GbfData6D " << std::endl << GbfComponent::toString();

    for ( int i = 0; i < toolTransforms.size(); i++)
    {
        stream << "toolHandle=" << std::setw(4) << static_cast<unsigned>(toolTransforms[i].toolHandle) << std::endl
               << "status=" << std::setw(4) << static_cast<unsigned>(toolTransforms[i].status) << (toolTransforms[i].isMissing() ? " - MISSING " : "")
                        << ", error=" << std::setw(2) << static_cast<unsigned>(toolTransforms[i].getErrorCode())
                        << " (" << TransformStatus::toString(toolTransforms[i].getErrorCode()) << ")" << std::endl
           << "transform=[q0, qx, qy, qz, tx, ty, tz, error ] = ["
           << toolTransforms[i].q0 << ","
           << toolTransforms[i].qx << ","
           << toolTransforms[i].qy << ","
           << toolTransforms[i].qz << ","
           << toolTransforms[i].tx << ","
           << toolTransforms[i].ty << ","
           << toolTransforms[i].tz << ","
           << toolTransforms[i].error << "]" << std::endl;
    }

    return stream.str();
}