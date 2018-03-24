#pragma once
#include <vector>
#include<memory>

#include "OVRafx.h"

#include "Camera.h"
#include "Material.h"
#include "AntTweakBar.h"
#include "DXRenderer.h"
#include "Entity.h"

class UIRenderer {

	private:

		static Camera*					s_Camera;
		static iVec2					s_resolution;
		static Mesh*					s_BBMesh;

		static std::vector<std::unique_ptr<Entity>>		s_EntitiesList;
		static std::map<std::string, Entity*>			s_EntityMap;

		static bool Initialize(ID3D11Device* device, ID3D11DeviceContext* devCtx, iVec2 size);
		static Mesh* CreateMesh();

	public:

		UIRenderer()  = delete;
		~UIRenderer() = delete;

		static bool Initialize(int width, int height);
		static bool CreateElement(std::string name, std::string mtlName, iVec2 dimensions, iVec2 position);
		static void Render(std::vector<std::unique_ptr<Entity>>& entities, std::vector<std::unique_ptr<Entity>>& scnEntities, std::vector<std::unique_ptr<Entity>>& tmpEntities);
		static void Update(float deltaTime, float worldTime);
		static bool UpdateElement(std::string name, iVec2 position, iVec2 dimensions) noexcept;
		static void Shutdown();
		static void Reset();

};

