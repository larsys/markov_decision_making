#The identifier of the sender Predicate Manager
uint32 pm_id
#A counter to help maintain consistenty between different PMs. Gets reset to 0 if it ever runs to UINT_MAX.
uint32 update_counter
#The set of true predicates
uint32[] true_predicates
#The set of rising predicates (i.e. true predicates that were previously false)
uint32[] rising_predicates
#The set of falling predicates (i.e. false predicates that were previously true)
uint32[] falling_predicates
