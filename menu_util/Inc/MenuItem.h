/*
 * MenuItem.h
 *
 *  Created on: Sep 21, 2022
 *      Author: ayeiser
 */

#ifndef MENUITEM_H_
#define MENUITEM_H_

#include "BufWriter.h"
#include "stdint.h"
#include "constants.h"
//#include "macros.h"

class MenuItem {
protected:
	MenuItem* parent;
	BufWriter& buf;
	char name[32];
	int name_len;
public:
	MenuItem(BufWriter &_buf, char* _name) :
		parent(nullptr), buf(_buf) {
		strncpy(name, _name, 32);
		name_len = strnlen(name, 31);
	}
	void setParent(MenuItem* _item) { parent = _item; };
	char* getName() { return name; };

	virtual MenuItem* up() = 0;
	virtual MenuItem* down() = 0;
	virtual MenuItem* left() = 0;
	virtual MenuItem* right() = 0;
	virtual MenuItem* enter() = 0;

	virtual void init() = 0;
	virtual void draw() = 0;
};

struct MotorData {
	float rpm, rpm_set, id, iq, i, vd, vq, v, vbus, power;
};

class RootMenu : public MenuItem {
protected:
	MenuItem* child;
	MotorData& data;
public:
	RootMenu(BufWriter& _buf, MotorData& _data) :
		MenuItem(_buf, "Root"), data(_data) {
		parent = this;
	};

	void link(MenuItem* _item) {
		child = _item;
		child->setParent(this);
	}

	MenuItem* up() { return enter(); };
	MenuItem* down() { return enter(); };
	MenuItem* left() { return enter(); };
	MenuItem* right() { return enter(); };
	MenuItem* enter() { child->init(); return child; };

	void init() {};
	void draw();
};

class Menu : public MenuItem {
protected:
	uint8_t len;
	int8_t index;
	MenuItem* children[16];
public:
	Menu(BufWriter &_buf, char* _name) :
		MenuItem(_buf, _name), len(0), index(0) {}

	void link(MenuItem*);

	MenuItem* up() {index = CLIP(index-1, 0, len); return this;}
	MenuItem* down() {index = CLIP(index+1, 0, len); return this;}
	MenuItem* left() {return parent;}
	MenuItem* right() {return enter();}
	MenuItem* enter();

	void init() {index = 0;}
	void draw();
};

class Entry : public MenuItem {
protected:
	int32_t* target;
	int32_t temp;
public:
	Entry(BufWriter &_buf, char* _name, int32_t* _target) :
		MenuItem(_buf, _name), target(_target) {}

	MenuItem* enter() {*target = temp; return parent;}
	void init() {temp = *target;}
};

class FlagEntry : public Entry {
protected:
	char lo[32];
	char hi[32];
	int32_t mask;
public:
	FlagEntry(BufWriter &_buf, char* _name,
			char* _lo, char* _hi,
			int32_t* _target, int32_t _mask) :
				Entry(_buf, _name, _target), mask(_mask) {
		strncpy(lo, _lo, 32);
		strncpy(hi, _hi, 32);
	}

	MenuItem* left() {return this;}
	MenuItem* right() {return this;}
	MenuItem* up() {temp |= mask; return this;}
	MenuItem* down() {temp &= (~mask); return this;}

	void draw();
};

class NumericEntry : public Entry {
protected:
	char prompt[32];
	char units[32];
	int32_t lower;
	int32_t upper;
	int32_t step;
public:
	NumericEntry(BufWriter &_buf, char* _name, int32_t* _target,
			char* _prompt, char* _units,
			int32_t _lower, int32_t _upper, int32_t _step=1) :
				Entry(_buf, _name, _target),
				lower(_lower), upper(_upper), step(_step) {
		strncpy(prompt, _prompt, 32);
		strncpy(units, _units, 32);
	}
	MenuItem* left() {return this;}
	MenuItem* right() {return this;}
	MenuItem* up() {temp = CLIP(temp+step, lower, upper); return this;}
	MenuItem* down() {temp = CLIP(temp-step, lower, upper); return this;}

	void draw();
};


#endif /* MENUITEM_H_ */
