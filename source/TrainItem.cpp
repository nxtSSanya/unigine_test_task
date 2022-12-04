#include "TrainItem.h"

#include "framework/engine.h"
#include "framework/utils.h"

TrainItem::TrainItem() {
	m_trainItemMesh = createCube();
}

Mesh TrainItem::getMesh() const {
	return m_trainItemMesh;
}
