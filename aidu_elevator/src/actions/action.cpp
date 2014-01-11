#include <aidu_elevator/actions/action.h>

aidu::elevator::Action::Action(ros::NodeHandle* nh) {
    this->nextAction = 0;
}

aidu::elevator::Action::~Action() {
    
}

void aidu::elevator::Action::execute() {
    
}

bool aidu::elevator::Action::finished() {
    return false;
}

aidu::elevator::Action* aidu::elevator::Action::getNextAction() {
    return this->nextAction;
}

void aidu::elevator::Action::setNextAction(aidu::elevator::Action* action) {
    this->nextAction = action;
}