#pragma once
#include <QThread>
class PCLVisualizer;

class render_thread : public QThread {
	Q_OBJECT

public:
	render_thread(QObject *parent);
	~render_thread();

	void setObj(PCLVisualizer* obj);

protected:
	void run();
private:
	PCLVisualizer* guiMain = nullptr;
};
