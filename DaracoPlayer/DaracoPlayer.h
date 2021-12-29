#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_DaracoPlayer.h"

class DaracoPlayer : public QMainWindow
{
    Q_OBJECT

public:
    DaracoPlayer(QWidget *parent = Q_NULLPTR);

private:
    Ui::DaracoPlayerClass ui;
};
