#include<render_thread.h>
#include<PCLVisualizer.h>



render_thread::render_thread(QObject *parent)
	: QThread(parent) {
}

render_thread::~render_thread() {
}

void render_thread::setObj(PCLVisualizer* obj) {
	guiMain = obj;
}

void render_thread::run() {
	while (true) {
		guiMain->run_render_thread();
	}
}


