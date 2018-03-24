#pragma once

// Skip some Windows extra ADTs etc
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

#include <iostream>
#include <string>
#include <unordered_map>
#include <algorithm>

template<class obj>
inline void CleanDelete(obj *& objectToDelete) {

	if (objectToDelete != nullptr) {

		delete objectToDelete;
		objectToDelete = nullptr;

	}

}

// Safe release COM
template<class Interface>
inline void CleanRelease(Interface *& pInterfaceToRelease) {

	if (pInterfaceToRelease != nullptr) {

		pInterfaceToRelease->Release();
		pInterfaceToRelease = nullptr;

	}

}

template<class T_KEY, class T_VAL>
bool QueryUnorderedMap(T_KEY key, T_VAL& output, const std::unordered_map<T_KEY, T_VAL>& map) {

	auto itemPair = map.find(key);

	if (itemPair != std::end(map)) {

		output = itemPair->second;

		return true;

	}

	return false;
};
