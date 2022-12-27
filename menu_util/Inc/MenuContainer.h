/*
 * MenuContainer.h
 *
 *  Created on: Sep 21, 2022
 *      Author: ayeiser
 */

#ifndef MENUCONTAINER_H_
#define MENUCONTAINER_H_

#include "main.h"
#include "Button.h"
#include "MenuItem.h"

#define SLEEP_TIME 200000

class MenuContainer {
private:
	MenuItem* root;
	MenuItem* current;
	Button& up, down, left, right, enter;
	uint32_t sleep_ctr;
public:
	MenuContainer(MenuItem* _root,
			Button& _up, Button& _down,
			Button& _left, Button& _right,
			Button& _enter) :
				root(_root), current(_root),
				up(_up), down(_down), left(_left), right(_right), enter(_enter),
				sleep_ctr(0) {}

	void update();
	void execute();
};

#endif /* MENUCONTAINER_H_ */
