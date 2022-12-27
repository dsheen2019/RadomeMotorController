/*
 * MenuContainer.cpp
 *
 *  Created on: Sep 21, 2022
 *      Author: ayeiser
 */

#include "MenuContainer.h"

void MenuContainer::update() {
	sleep_ctr++;
	up.update();
	down.update();
	left.update();
	right.update();
	enter.update();
}

void MenuContainer::execute() {
	if (left.getRisingEdge()) {
		current = current->left();
		sleep_ctr = 0;
	}
	if (right.getRisingEdge()) {
		current = current->right();
		sleep_ctr = 0;
	}
	if (up.getRisingEdge()) {
		current = current->up();
		sleep_ctr = 0;
	}
	if (down.getRisingEdge()) {
		current = current->down();
		sleep_ctr = 0;
	}
	if (enter.getRisingEdge()) {
		current = current->enter();
		sleep_ctr = 0;
	}
	if (sleep_ctr > SLEEP_TIME) {
		sleep_ctr = SLEEP_TIME - 25000;
		current = root;
	}
	current->draw();
}
