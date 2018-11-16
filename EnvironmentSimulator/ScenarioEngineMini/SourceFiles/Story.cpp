#include "Story.hpp"
#include "CommonMini.hpp"


Story::Story()
{
	LOG("Story: New Story created");
}

void Story::Print()
{
	LOG("Story: %s", name_.c_str());
}

void Story::Step(double dt)
{
//	for(int i=0; i<Act)
}

