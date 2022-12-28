/*
 * MenuItem.cpp
 *
 *  Created on: Sep 21, 2022
 *      Author: ayeiser
 */

#include "MenuItem.h"

void RootMenu::draw() {
	buf.clear();
	int n;
	char str[64];
	n = sprintf(str, "(%5d)%5d RPM", int(data.rpm_set), int(data.rpm));
	buf.writeLine8x16(str, n);
	buf.newline();

	n = sprintf(str, "Id=% 6.3f Iq=% 6.3fA", data.id, data.iq);
	buf.writeLine6x8(str, n);

	n = sprintf(str, "Vd=% 6.2f Vq=% 6.2fV", data.vd, data.vq);
	buf.writeLine6x8(str, n);

	n = sprintf(str, "Power=% 6.2fW", data.power);
	buf.writeLine6x8(str, n);

	n = sprintf(str, "Vbus=% 6.2fV", data.vbus);
	buf.writeLine6x8(str, n);
}

void Menu::link(MenuItem* _child) {
	if (len < 16) {
		children[len] = _child;
		_child->setParent(this);
		len++;
	}
}

MenuItem* Menu::enter() {
	if(index == 0 || index > len) {
		return parent;
	} else {
		return children[index-1];
	}
}

void Menu::draw() {
	buf.clear();
	buf.writeLine8x16(name, name_len);
	buf.newline();
	int8_t k = MIN(index+2, len);
	int8_t j = MAX(k-4, 0);
	for (int8_t i = 0; j<=k; i++, j++) {
		char c = (j == index) ? '>' : ' ';
		char str[64];
		memset(str, 0, 32);
		int n;
		if (j==0) {
			n = sprintf(str, "%c Back", c);
		} else {
			n = sprintf(str, "%c %s", c, children[j-1]->getName());
		}
		buf.writeLine6x8(str, n);
	}
}

void FlagEntry::draw() {
	buf.clear();
	buf.writeLine8x16(name);
	buf.newline();
	if (temp) {
		buf.writeStr6x8(hi, 32);
	} else {
		buf.writeStr6x8(lo, 32);
	}
}

void NumericEntry::draw() {
	buf.clear();
	buf.writeLine8x16(name);
	buf.newline();
	char str[128];
	int len = sprintf(str, "%s: %d %s", prompt, int(temp), units);
	buf.writeStr6x8(str, len);
}

