/********************************************************************************
** Form generated from reading UI file 'PCLVisualizer.ui'
**
** Created by: Qt User Interface Compiler version 5.12.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PCLVISUALIZER_H
#define UI_PCLVISUALIZER_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_PCLVisualizerClass
{
public:
    QWidget *centralWidget;
    QGridLayout *gridLayout;
    QLineEdit *mainfest_url;
    QVTKWidget *qvtkWidget;
    QPushButton *button_play_notile;
    QPushButton *button_play_Tile;
    QLineEdit *scheme;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *PCLVisualizerClass)
    {
        if (PCLVisualizerClass->objectName().isEmpty())
            PCLVisualizerClass->setObjectName(QString::fromUtf8("PCLVisualizerClass"));
        PCLVisualizerClass->resize(1014, 663);
        centralWidget = new QWidget(PCLVisualizerClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        gridLayout = new QGridLayout(centralWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        mainfest_url = new QLineEdit(centralWidget);
        mainfest_url->setObjectName(QString::fromUtf8("mainfest_url"));
        mainfest_url->setMinimumSize(QSize(933, 20));

        gridLayout->addWidget(mainfest_url, 0, 0, 1, 3);

        qvtkWidget = new QVTKWidget(centralWidget);
        qvtkWidget->setObjectName(QString::fromUtf8("qvtkWidget"));

        gridLayout->addWidget(qvtkWidget, 1, 0, 1, 3);

        button_play_notile = new QPushButton(centralWidget);
        button_play_notile->setObjectName(QString::fromUtf8("button_play_notile"));
        button_play_notile->setFlat(false);

        gridLayout->addWidget(button_play_notile, 2, 0, 1, 1);

        button_play_Tile = new QPushButton(centralWidget);
        button_play_Tile->setObjectName(QString::fromUtf8("button_play_Tile"));
        button_play_Tile->setFlat(false);

        gridLayout->addWidget(button_play_Tile, 2, 1, 1, 1);

        scheme = new QLineEdit(centralWidget);
        scheme->setObjectName(QString::fromUtf8("scheme"));

        gridLayout->addWidget(scheme, 2, 2, 1, 1);

        PCLVisualizerClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(PCLVisualizerClass);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1014, 23));
        PCLVisualizerClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(PCLVisualizerClass);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        PCLVisualizerClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(PCLVisualizerClass);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        PCLVisualizerClass->setStatusBar(statusBar);

        retranslateUi(PCLVisualizerClass);

        QMetaObject::connectSlotsByName(PCLVisualizerClass);
    } // setupUi

    void retranslateUi(QMainWindow *PCLVisualizerClass)
    {
        PCLVisualizerClass->setWindowTitle(QApplication::translate("PCLVisualizerClass", "PCLVisualizer", nullptr));
        mainfest_url->setInputMask(QString());
        mainfest_url->setText(QApplication::translate("PCLVisualizerClass", "http://localhost/longdress/mainfest.xml", nullptr));
        button_play_notile->setText(QApplication::translate("PCLVisualizerClass", "NoTile", nullptr));
        button_play_Tile->setText(QApplication::translate("PCLVisualizerClass", "Tile", nullptr));
    } // retranslateUi

};

namespace Ui {
    class PCLVisualizerClass: public Ui_PCLVisualizerClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PCLVISUALIZER_H
