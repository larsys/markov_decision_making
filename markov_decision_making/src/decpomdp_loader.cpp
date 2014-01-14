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
#include <madp/StateFactorDiscreteLALA.h>



#endif

#include <sys/stat.h>

#include <markov_decision_making/SymbolMetadata.h>
#include <markov_decision_making/FactoredSymbolMetadata.h>
#include <markov_decision_making/FactoredDistribution.h>
#include <markov_decision_making/decpomdp_loader.h>



using namespace ros;
using namespace std;
using namespace markov_decision_making;



#ifdef HAVE_MADP



DecPOMDPLoader::
DecPOMDPLoader (const string& problem_file) :
  action_metadata_pub_ (nh_.advertise<SymbolMetadata> ("action_metadata", 1, true)),
  state_metadata_pub_ (nh_.advertise<FactoredSymbolMetadata> ("state_metadata", 1, true)),
  observation_metadata_pub_ (nh_.advertise<SymbolMetadata> ("observation_metadata", 1, true)),
  initial_state_distribution_pub_ (nh_.advertise<FactoredDistribution> ("initial_state_distribution", 1, false))
{
  string ext = problem_file.substr (problem_file.find_last_of ("."));
  
  try {
    if (ext == ".pgmx") {
      boost::shared_ptr<FactoredDecPOMDPDiscrete> f (new FactoredDecPOMDPDiscrete ("", "", problem_file));
      MADPParser parser (f.get());
      bool isSparse = false;
      bool cacheFlatModels = false;
      int marginalizeStateFactor;
      
      NodeHandle private_nh ("~");
      private_nh.getParam ("is_sparse", isSparse);
      private_nh.getParam ("cache_flat_models", cacheFlatModels);
      
      if (private_nh.getParam ("marginalize", marginalizeStateFactor)) {
        f->MarginalizeStateFactor (marginalizeStateFactor, isSparse);  //TODO: NOTE: This is necessary for standard Perseus, but needs to be faster.
      }
      else if (cacheFlatModels) {
        f->CacheFlatModels (isSparse);
      }
      publishStateMetadata (f);
      publishInitialStateDistribution (f);
      decpomdp_ = f;
    }
    else if (ext == ".dpomdp") {
      boost::shared_ptr<DecPOMDPDiscrete> d (new DecPOMDPDiscrete ("", "", problem_file));
      MADPParser parser (d.get());
      publishStateMetadata (d);
      publishInitialStateDistribution (d);
      decpomdp_ = d;
    }
    else {
      ROS_ERROR_STREAM ("Unsupported model format \"" << ext << "\"");
      abort();
    }
    publishActionMetadata();
    publishObservationMetadata ();
  }
  catch (E& e) {
    e.Print();
    abort();
  }
}



void
DecPOMDPLoader::
publishActionMetadata ()
{
  ActionMetadata team_metadata;
  for (Index ag = 0; ag < decpomdp_->GetNrAgents(); ag++) {
    AgentActionMetadata ag_metadata;
    uint32_t nr_actions = decpomdp_->GetNrActions (ag);
    ag_metadata.number_of_actions = nr_actions;
    for (Index action = 0; action < nr_actions; action++) {
      ag_metadata.action_names.push_back (decpomdp_->GetAction (ag, action)->GetName());
    }
    team_metadata.agent_actions.push_back (ag_metadata);
  }
  action_metadata_pub_.publish (team_metadata);
}



void
DecPOMDPLoader::
publishStateMetadata (boost::shared_ptr<FactoredDecPOMDPDiscrete> f)
{
  StateMetadata state_metadata;
  for (Index k = 0; k < f->GetNrStateFactors(); k++) {
    StateFactorMetadata factor_metadata;
    const StateFactorDiscrete* sf = f->GetStateFactorDiscrete (k);
    factor_metadata.factor_name = sf->GetName();
    for (Index j = 0; j < f->GetNrValuesForFactor (k); j++) {
      factor_metadata.value_names.push_back (sf->GetStateFactorValue (j));
    }
    state_metadata.factor_metadata.push_back (factor_metadata);
  }
  state_metadata_pub_.publish (state_metadata);
}



void
DecPOMDPLoader::
publishStateMetadata (boost::shared_ptr<DecPOMDPDiscrete> d)
{
  StateMetadata state_metadata;
  StateFactorMetadata factor_metadata;
  factor_metadata.factor_name = "Joint State";
  for (Index k = 0; k < d->GetNrStates (); k++) {
    factor_metadata.value_names.push_back (d->GetState (k)->GetName());
  }
  state_metadata.factor_metadata.push_back (factor_metadata);
  state_metadata_pub_.publish (state_metadata);
}



void
DecPOMDPLoader::
publishObservationMetadata ()
{
  ///Note: Fully-observable systems in MADP have empty observation models. In that case, this metadata should be a vector of zeros.
  ObservationMetadata msg;
  for (Index k = 0; k < decpomdp_->GetNrAgents(); k++) {
    msg.number_of_observations.push_back (decpomdp_->GetNrObservations (k));
  }
  observation_metadata_pub_.publish (msg);
}



void
DecPOMDPLoader::
publishInitialStateDistribution (boost::shared_ptr<FactoredDecPOMDPDiscrete> f)
{
  FSDist_COF* fsd = (FSDist_COF*) f->GetFactoredISD();
  FactoredDistribution fdist;
  for (size_t i = 0; i < f->GetNrStateFactors(); i++) {
    BeliefStateInfo b;
    for (size_t j = 0; j < f->GetNrValuesForFactor (i); j++) {
      b.belief.push_back (fsd->GetReferrence (i, j));
    }
    fdist.factors.push_back (b);
  }
  initial_state_distribution_pub_.publish (fdist);
}



void
DecPOMDPLoader::
publishInitialStateDistribution (boost::shared_ptr<DecPOMDPDiscrete> d)
{
  vector<double> isd = d->GetISD()->ToVectorOfDoubles();
  FactoredDistribution fdist;
  BeliefStateInfo b;
  b.belief = isd;
  fdist.factors.push_back (b);
  initial_state_distribution_pub_.publish (b);
}



const boost::shared_ptr<DecPOMDPDiscreteInterface>
DecPOMDPLoader::
GetDecPOMDP()
{
  return decpomdp_;
}



#else //NO MADP -- This constructor throws an error



DecPOMDPLoader::
DecPOMDPLoader (const string& problem_file)
{
  ROS_ERROR_STREAM ("MDM requires MADP to parse problem files.");
  ROS_ERROR_STREAM ("Please install MADP and recompile MDM if you require this functionality.");
  abort();
}



#endif
