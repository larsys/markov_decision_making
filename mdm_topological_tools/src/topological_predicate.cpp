/**\file topological_predicate.cpp
 *
 * Author:
 * Joao Messias <jmessias@isr.ist.utl.pt>
 *
 * TopologicalTools is a set of utilities to aid the deployment of the MDM library
 * in topological navigation problems.
 * Copyright (C) 2014 Instituto Superior Tecnico, Instituto de Sistemas e Robotica
 *
 * This file is part of TopologicalTools.
 *
 * TopologicalTools is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * TopologicalTools is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <mdm_topological_tools/topological_predicate.h>


using namespace std;
using namespace ros;
using namespace predicate_manager;
using namespace mdm_topological_tools;


TopologicalPredicate::
TopologicalPredicate ( const string& label_topic,
                       const string& name ) :
    Predicate ( name ),
    label_subs_ ( nh_.subscribe ( label_topic, 1, &TopologicalPredicate::labelCallback, this ) )
{
    nh_.getParam ( "predicate_labels/" + getName(), ( int& ) target_label_ );
}

TopologicalPredicate::TopologicalPredicate ( const string& label_topic,
        const string& name,
        const uint32_t target_label ) :
    Predicate ( name ),
    label_subs_ ( nh_.subscribe ( label_topic, 1, &TopologicalPredicate::labelCallback, this ) ),
    target_label_ ( target_label )
{}

void TopologicalPredicate::labelCallback ( const PoseLabelConstPtr& msg )
{
    received_label_ = msg->label;
    update();
}

void TopologicalPredicate::update()
{
    setValue ( received_label_ == target_label_ );
}
