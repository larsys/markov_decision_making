/**\file decpomdp_loader.cpp
 *
 * Author:
 * Joao Messias <jmessias@isr.ist.utl.pt>
 *
 * Markov Decision Making is a ROS library for robot decision-making based on MDPs.
 * Copyright (C) 2014 Instituto Superior Tecnico, Instituto de Sistemas e Robotica
 *
 * This file is part of Markov Decision Making.
 *
 * Markov Decision Making is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Markov Decision Making is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */



#ifdef HAVE_MADP

#include <madp/NullPlanner.h>
#include <madp/MADPParser.h>
#include <madp/StateFactorDiscrete.h>



#endif

#include <sys/stat.h>

#include <mdm_library/SymbolMetadata.h>
#include <mdm_library/FactoredSymbolMetadata.h>
#include <mdm_library/FactoredDistribution.h>
#include <mdm_library/decpomdp_loader.h>



using namespace ros;
using namespace std;
using namespace mdm_library;



#ifdef HAVE_MADP



DecPOMDPLoader::
DecPOMDPLoader ( const string& problem_file ) :
    action_metadata_pub_ ( nh_.advertise<FactoredSymbolMetadata> ( "action_metadata", 1, true ) ),
    state_metadata_pub_ ( nh_.advertise<FactoredSymbolMetadata> ( "state_metadata", 1, true ) ),
    observation_metadata_pub_ ( nh_.advertise<FactoredSymbolMetadata> ( "observation_metadata", 1, true ) ),
    initial_state_distribution_pub_ ( nh_.advertise<FactoredDistribution> ( "initial_state_distribution", 1, false ) )
{
    string ext = problem_file.substr ( problem_file.find_last_of ( "." ) );

    try
    {
        if ( ext == ".pgmx" )
        {
            boost::shared_ptr<FactoredDecPOMDPDiscrete> f ( new FactoredDecPOMDPDiscrete ( "", "", problem_file ) );
            MADPParser parser ( f.get() );
            bool isSparse = false;
            bool cacheFlatModels = false;
            int marginalizeStateFactor;

            NodeHandle private_nh ( "~" );
            private_nh.getParam ( "is_sparse", isSparse );
            private_nh.getParam ( "cache_flat_models", cacheFlatModels );

            if ( private_nh.getParam ( "marginalize", marginalizeStateFactor ) )
            {
                f->MarginalizeStateFactor ( marginalizeStateFactor, isSparse ); //TODO: NOTE: This is necessary for standard Perseus, but needs to be faster.
            }
            else if ( cacheFlatModels )
            {
                f->CacheFlatModels ( isSparse );
            }
            publishStateMetadata ( f );
            publishInitialStateDistribution ( f );
            decpomdp_ = f;
        }
        else if ( ext == ".dpomdp" )
        {
            boost::shared_ptr<DecPOMDPDiscrete> d ( new DecPOMDPDiscrete ( "", "", problem_file ) );
            MADPParser parser ( d.get() );
            publishStateMetadata ( d );
            publishInitialStateDistribution ( d );
            decpomdp_ = d;
        }
        else
        {
            ROS_ERROR_STREAM ( "Unsupported model format \"" << ext << "\"" );
            abort();
        }
        publishActionMetadata();
        publishObservationMetadata ();
    }
    catch ( E& e )
    {
        e.Print();
        abort();
    }
}



void
DecPOMDPLoader::
publishActionMetadata ()
{
    FactoredSymbolMetadata team_metadata;
    for ( uint32_t ag = 0; ag < decpomdp_->GetNrAgents(); ag++ )
    {
        SymbolMetadata ag_metadata;
        uint32_t nr_actions = decpomdp_->GetNrActions ( ag );
        ag_metadata.number_of_symbols = nr_actions;
        for ( uint32_t action = 0; action < nr_actions; action++ )
        {
            ag_metadata.symbol_names.push_back ( decpomdp_->GetAction ( ag, action )->GetName() );
        }
        team_metadata.factors.push_back ( ag_metadata );
    }
    action_metadata_pub_.publish ( team_metadata );
}



void
DecPOMDPLoader::
publishStateMetadata ( boost::shared_ptr<FactoredDecPOMDPDiscrete> f )
{
    FactoredSymbolMetadata state_metadata;
    for ( uint32_t k = 0; k < f->GetNrStateFactors(); k++ )
    {
        SymbolMetadata factor_metadata;
        const StateFactorDiscrete* sf = f->GetStateFactorDiscrete ( k );
        state_metadata.factor_names.push_back ( sf->GetName() );
        for ( uint32_t j = 0; j < f->GetNrValuesForFactor ( k ); j++ )
        {
            factor_metadata.symbol_names.push_back ( sf->GetStateFactorValue ( j ) );
        }
        state_metadata.factors.push_back ( factor_metadata );
    }
    state_metadata_pub_.publish ( state_metadata );
}



void
DecPOMDPLoader::
publishStateMetadata ( boost::shared_ptr<DecPOMDPDiscrete> d )
{
    FactoredSymbolMetadata state_metadata;
    SymbolMetadata symbol_metadata;
    state_metadata.factor_names.push_back ( "Joint State" );
    for ( uint32_t k = 0; k < d->GetNrStates (); k++ )
    {
        symbol_metadata.symbol_names.push_back ( d->GetState ( k )->GetName() );
    }
    symbol_metadata.number_of_symbols = d->GetNrStates();
    state_metadata.factors.push_back ( symbol_metadata );
    state_metadata_pub_.publish ( state_metadata );
}



void
DecPOMDPLoader::
publishObservationMetadata ()
{
    ///Note: Fully-observable systems in MADP have empty observation models. In that case, this metadata should be a vector of zeros.
    FactoredSymbolMetadata team_metadata;
    for ( uint32_t ag = 0; ag < decpomdp_->GetNrAgents(); ag++ )
    {
        SymbolMetadata ag_metadata;
        uint32_t nr_observations = decpomdp_->GetNrObservations ( ag );
        ag_metadata.number_of_symbols = nr_observations;
        for ( uint32_t observation = 0; observation < nr_observations; observation++ )
        {
            ag_metadata.symbol_names.push_back ( decpomdp_->GetObservation ( ag, observation )->GetName() );
        }
        team_metadata.factors.push_back ( ag_metadata );
    }
    observation_metadata_pub_.publish ( team_metadata );
}



void
DecPOMDPLoader::
publishInitialStateDistribution ( boost::shared_ptr<FactoredDecPOMDPDiscrete> f )
{
    FSDist_COF* fsd = ( FSDist_COF* ) f->GetFactoredISD();
    FactoredDistribution fdist;
    for ( size_t i = 0; i < f->GetNrStateFactors(); i++ )
    {
        BeliefStateInfo b;
        for ( size_t j = 0; j < f->GetNrValuesForFactor ( i ); j++ )
        {
            b.belief.push_back ( fsd->GetReferrence ( i, j ) );
        }
        fdist.factors.push_back ( b );
    }
    initial_state_distribution_pub_.publish ( fdist );
}



void
DecPOMDPLoader::
publishInitialStateDistribution ( boost::shared_ptr<DecPOMDPDiscrete> d )
{
    vector<double> isd = d->GetISD()->ToVectorOfDoubles();
    FactoredDistribution fdist;
    BeliefStateInfo b;
    b.belief = isd;
    fdist.factors.push_back ( b );
    initial_state_distribution_pub_.publish ( fdist );
}



const boost::shared_ptr<DecPOMDPDiscreteInterface>
DecPOMDPLoader::
GetDecPOMDP()
{
    return decpomdp_;
}



#else //NO MADP -- This constructor throws an error



DecPOMDPLoader::
DecPOMDPLoader ( const string& problem_file )
{
    ROS_ERROR_STREAM ( "MDM requires MADP to parse problem files." );
    ROS_ERROR_STREAM ( "Please install MADP and recompile MDM if you require this functionality." );
    abort();
}



#endif
