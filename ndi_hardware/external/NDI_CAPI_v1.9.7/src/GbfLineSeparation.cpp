//----------------------------------------------------------------------------
//
//  Copyright (C) 2022, Northern Digital Inc. All rights reserved.
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
#include <cmath>

#include "GbfLineSeparation.h"

#ifndef NAN
#define NAN -1E300
#endif

GbfLineSeparation::GbfLineSeparation( BufferedReader& reader, int numberOfTools )
{
    uint16_t toolHandle = 0x00;
    uint16_t numberOfLineSeparations = 0x00;
    for ( int i = 0; i < numberOfTools; i++ )
    {
        // Read the data
        toolHandle = reader.get_uint16();
        numberOfLineSeparations = reader.get_uint16();

        // Put the toolHandle into its own vector
        toolHandles.push_back( toolHandle );

        // Create a corresponding vector with the line separation values
        std::vector<double> listLineSeparations;
        for ( int i = 0; i < numberOfLineSeparations; i++ )
        {
            // Read the marker index
            int markerIndex = reader.get_uint16();
            while ( listLineSeparations.size() <= markerIndex )
            {
                listLineSeparations.push_back( NAN );
            }

            // Read the line separation
            double lineSeparation  = reader.get_double();
            listLineSeparations.at( markerIndex ) = lineSeparation;
        }
        // Add the list of line separations for this tool handle to the master list
        lineSeparations.push_back( listLineSeparations );
    }
}

std::string GbfLineSeparation::toString() const
{
    std::stringstream stream;
    stream << std::hex << std::setfill( '0' ) << std::setprecision( 4 );
    stream << "----GbfLineSeparation " << std::endl << GbfComponent::toString();

    // For each tool, print the tool handle and all of its associated errors
    for ( int toolIndex = 0; toolIndex < toolHandles.size(); toolIndex++ )
    {
        stream << "toolHandleReference=" << std::setw( 4 ) << toolHandles[toolIndex] << std::endl
            << "line separations=[";
        for ( int i = 0; i < lineSeparations[toolIndex].size(); i++ )
        {
            stream << " " << lineSeparations[toolIndex][i];
        }
        stream << " ]" << std::endl;

    }
    return stream.str();
}