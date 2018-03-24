#pragma once

#include "Kinectafx.h"

#include <vector>

class KinectV2IRFrame {

	private:

		std::vector<UINT16>	data;

	public:

		KinectV2IRFrame(std::vector<UINT16>& arr) : data(arr) { }
		KinectV2IRFrame() { }
		~KinectV2IRFrame() { }

		void Unpack(std::vector<UINT16>& arr) { arr = data; }

};
