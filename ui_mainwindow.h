/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Tue Oct 14 15:54:17 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QFrame>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include "glpanel.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionExit;
    QAction *actionReset;
    QAction *actionReset_Everything;
    QWidget *centralWidget;
    GLPanel *GLWidget;
    QFrame *parameterFrame;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QGroupBox *simOptionsBox;
    QWidget *horizontalLayoutWidget;
    QHBoxLayout *horizontalLayout;
    QGroupBox *SimulationBox;
    QPushButton *startSimulationButton;
    QGroupBox *SimParametersBox;
    QLabel *timeStepLabel;
    QLabel *newtonTolLabel;
    QLabel *newtonMaxItersLabel;
    QLineEdit *timeStepEdit;
    QLineEdit *newtonTolEdit;
    QLineEdit *newtonMaxItersEdit;
    QGroupBox *activeForcesBox;
    QCheckBox *gravityCheckBox;
    QCheckBox *floorCheckBox;
    QLabel *gravityGLabel;
    QLineEdit *gravityGEdit;
    QLabel *floorStiffness;
    QLineEdit *floorStiffnessEdit;
    QGroupBox *uiOptionsBox;
    QWidget *horizontalLayoutWidget_2;
    QHBoxLayout *horizontalLayout_2;
    QVBoxLayout *verticalLayout_2;
    QGroupBox *rigidBodyTypeBox;
    QRadioButton *sphereButton;
    QRadioButton *twoByFourButton;
    QRadioButton *bunnyButton;
    QRadioButton *customButton;
    QGroupBox *launchOptionsBox;
    QLabel *launchVelLabel;
    QLineEdit *launchVelEdit;
    QCheckBox *randomOrienatationCheckBox;
    QCheckBox *randomAngularVelCheckBox;
    QLabel *densityLabel;
    QLineEdit *densityEdit;
    QLabel *randomVelMagLabel;
    QLineEdit *randomVelMagEdit;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuScene;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1200, 800);
        actionExit = new QAction(MainWindow);
        actionExit->setObjectName(QString::fromUtf8("actionExit"));
        actionReset = new QAction(MainWindow);
        actionReset->setObjectName(QString::fromUtf8("actionReset"));
        actionReset_Everything = new QAction(MainWindow);
        actionReset_Everything->setObjectName(QString::fromUtf8("actionReset_Everything"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        GLWidget = new GLPanel(centralWidget);
        GLWidget->setObjectName(QString::fromUtf8("GLWidget"));
        GLWidget->setGeometry(QRect(10, 0, 731, 731));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(GLWidget->sizePolicy().hasHeightForWidth());
        GLWidget->setSizePolicy(sizePolicy);
        GLWidget->setFocusPolicy(Qt::StrongFocus);
        parameterFrame = new QFrame(centralWidget);
        parameterFrame->setObjectName(QString::fromUtf8("parameterFrame"));
        parameterFrame->setGeometry(QRect(749, -1, 441, 731));
        parameterFrame->setFrameShape(QFrame::StyledPanel);
        parameterFrame->setFrameShadow(QFrame::Raised);
        verticalLayoutWidget = new QWidget(parameterFrame);
        verticalLayoutWidget->setObjectName(QString::fromUtf8("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(9, -1, 431, 731));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        simOptionsBox = new QGroupBox(verticalLayoutWidget);
        simOptionsBox->setObjectName(QString::fromUtf8("simOptionsBox"));
        simOptionsBox->setMaximumSize(QSize(16777215, 220));
        horizontalLayoutWidget = new QWidget(simOptionsBox);
        horizontalLayoutWidget->setObjectName(QString::fromUtf8("horizontalLayoutWidget"));
        horizontalLayoutWidget->setGeometry(QRect(9, 19, 421, 181));
        horizontalLayout = new QHBoxLayout(horizontalLayoutWidget);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        SimulationBox = new QGroupBox(horizontalLayoutWidget);
        SimulationBox->setObjectName(QString::fromUtf8("SimulationBox"));
        startSimulationButton = new QPushButton(SimulationBox);
        startSimulationButton->setObjectName(QString::fromUtf8("startSimulationButton"));
        startSimulationButton->setGeometry(QRect(10, 50, 181, 27));

        horizontalLayout->addWidget(SimulationBox);

        SimParametersBox = new QGroupBox(horizontalLayoutWidget);
        SimParametersBox->setObjectName(QString::fromUtf8("SimParametersBox"));
        timeStepLabel = new QLabel(SimParametersBox);
        timeStepLabel->setObjectName(QString::fromUtf8("timeStepLabel"));
        timeStepLabel->setGeometry(QRect(10, 30, 81, 21));
        newtonTolLabel = new QLabel(SimParametersBox);
        newtonTolLabel->setObjectName(QString::fromUtf8("newtonTolLabel"));
        newtonTolLabel->setGeometry(QRect(10, 50, 131, 21));
        newtonMaxItersLabel = new QLabel(SimParametersBox);
        newtonMaxItersLabel->setObjectName(QString::fromUtf8("newtonMaxItersLabel"));
        newtonMaxItersLabel->setGeometry(QRect(10, 70, 131, 21));
        timeStepEdit = new QLineEdit(SimParametersBox);
        timeStepEdit->setObjectName(QString::fromUtf8("timeStepEdit"));
        timeStepEdit->setGeometry(QRect(140, 30, 61, 21));
        newtonTolEdit = new QLineEdit(SimParametersBox);
        newtonTolEdit->setObjectName(QString::fromUtf8("newtonTolEdit"));
        newtonTolEdit->setGeometry(QRect(140, 50, 61, 21));
        newtonMaxItersEdit = new QLineEdit(SimParametersBox);
        newtonMaxItersEdit->setObjectName(QString::fromUtf8("newtonMaxItersEdit"));
        newtonMaxItersEdit->setGeometry(QRect(140, 70, 61, 21));

        horizontalLayout->addWidget(SimParametersBox);


        verticalLayout->addWidget(simOptionsBox);

        activeForcesBox = new QGroupBox(verticalLayoutWidget);
        activeForcesBox->setObjectName(QString::fromUtf8("activeForcesBox"));
        activeForcesBox->setMaximumSize(QSize(16777215, 170));
        gravityCheckBox = new QCheckBox(activeForcesBox);
        gravityCheckBox->setObjectName(QString::fromUtf8("gravityCheckBox"));
        gravityCheckBox->setGeometry(QRect(30, 30, 97, 21));
        floorCheckBox = new QCheckBox(activeForcesBox);
        floorCheckBox->setObjectName(QString::fromUtf8("floorCheckBox"));
        floorCheckBox->setGeometry(QRect(30, 50, 97, 21));
        gravityGLabel = new QLabel(activeForcesBox);
        gravityGLabel->setObjectName(QString::fromUtf8("gravityGLabel"));
        gravityGLabel->setGeometry(QRect(230, 30, 121, 21));
        gravityGEdit = new QLineEdit(activeForcesBox);
        gravityGEdit->setObjectName(QString::fromUtf8("gravityGEdit"));
        gravityGEdit->setGeometry(QRect(370, 30, 51, 21));
        floorStiffness = new QLabel(activeForcesBox);
        floorStiffness->setObjectName(QString::fromUtf8("floorStiffness"));
        floorStiffness->setGeometry(QRect(230, 50, 121, 21));
        floorStiffnessEdit = new QLineEdit(activeForcesBox);
        floorStiffnessEdit->setObjectName(QString::fromUtf8("floorStiffnessEdit"));
        floorStiffnessEdit->setGeometry(QRect(370, 50, 51, 21));

        verticalLayout->addWidget(activeForcesBox);

        uiOptionsBox = new QGroupBox(verticalLayoutWidget);
        uiOptionsBox->setObjectName(QString::fromUtf8("uiOptionsBox"));
        horizontalLayoutWidget_2 = new QWidget(uiOptionsBox);
        horizontalLayoutWidget_2->setObjectName(QString::fromUtf8("horizontalLayoutWidget_2"));
        horizontalLayoutWidget_2->setGeometry(QRect(10, 20, 421, 231));
        horizontalLayout_2 = new QHBoxLayout(horizontalLayoutWidget_2);
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(0, 0, 0, 0);
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        rigidBodyTypeBox = new QGroupBox(horizontalLayoutWidget_2);
        rigidBodyTypeBox->setObjectName(QString::fromUtf8("rigidBodyTypeBox"));
        rigidBodyTypeBox->setMaximumSize(QSize(16777215, 130));
        sphereButton = new QRadioButton(rigidBodyTypeBox);
        sphereButton->setObjectName(QString::fromUtf8("sphereButton"));
        sphereButton->setGeometry(QRect(10, 30, 117, 21));
        twoByFourButton = new QRadioButton(rigidBodyTypeBox);
        twoByFourButton->setObjectName(QString::fromUtf8("twoByFourButton"));
        twoByFourButton->setGeometry(QRect(10, 50, 117, 21));
        bunnyButton = new QRadioButton(rigidBodyTypeBox);
        bunnyButton->setObjectName(QString::fromUtf8("bunnyButton"));
        bunnyButton->setGeometry(QRect(10, 70, 117, 21));
        customButton = new QRadioButton(rigidBodyTypeBox);
        customButton->setObjectName(QString::fromUtf8("customButton"));
        customButton->setGeometry(QRect(10, 90, 117, 21));

        verticalLayout_2->addWidget(rigidBodyTypeBox);

        launchOptionsBox = new QGroupBox(horizontalLayoutWidget_2);
        launchOptionsBox->setObjectName(QString::fromUtf8("launchOptionsBox"));
        launchOptionsBox->setMinimumSize(QSize(0, 0));
        launchOptionsBox->setBaseSize(QSize(0, 0));
        launchVelLabel = new QLabel(launchOptionsBox);
        launchVelLabel->setObjectName(QString::fromUtf8("launchVelLabel"));
        launchVelLabel->setGeometry(QRect(10, 50, 111, 17));
        launchVelEdit = new QLineEdit(launchOptionsBox);
        launchVelEdit->setObjectName(QString::fromUtf8("launchVelEdit"));
        launchVelEdit->setGeometry(QRect(130, 50, 51, 21));
        randomOrienatationCheckBox = new QCheckBox(launchOptionsBox);
        randomOrienatationCheckBox->setObjectName(QString::fromUtf8("randomOrienatationCheckBox"));
        randomOrienatationCheckBox->setGeometry(QRect(10, 70, 181, 22));
        randomAngularVelCheckBox = new QCheckBox(launchOptionsBox);
        randomAngularVelCheckBox->setObjectName(QString::fromUtf8("randomAngularVelCheckBox"));
        randomAngularVelCheckBox->setGeometry(QRect(10, 90, 171, 22));
        densityLabel = new QLabel(launchOptionsBox);
        densityLabel->setObjectName(QString::fromUtf8("densityLabel"));
        densityLabel->setGeometry(QRect(10, 30, 111, 17));
        densityEdit = new QLineEdit(launchOptionsBox);
        densityEdit->setObjectName(QString::fromUtf8("densityEdit"));
        densityEdit->setGeometry(QRect(130, 30, 51, 21));
        randomVelMagLabel = new QLabel(launchOptionsBox);
        randomVelMagLabel->setObjectName(QString::fromUtf8("randomVelMagLabel"));
        randomVelMagLabel->setGeometry(QRect(220, 90, 111, 17));
        randomVelMagEdit = new QLineEdit(launchOptionsBox);
        randomVelMagEdit->setObjectName(QString::fromUtf8("randomVelMagEdit"));
        randomVelMagEdit->setGeometry(QRect(310, 90, 51, 21));

        verticalLayout_2->addWidget(launchOptionsBox);


        horizontalLayout_2->addLayout(verticalLayout_2);


        verticalLayout->addWidget(uiOptionsBox);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1200, 25));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        menuScene = new QMenu(menuBar);
        menuScene->setObjectName(QString::fromUtf8("menuScene"));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuScene->menuAction());
        menuFile->addAction(actionExit);
        menuScene->addAction(actionReset);
        menuScene->addAction(actionReset_Everything);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Furious Birds", 0, QApplication::UnicodeUTF8));
        actionExit->setText(QApplication::translate("MainWindow", "Exit", 0, QApplication::UnicodeUTF8));
        actionReset->setText(QApplication::translate("MainWindow", "Clear Scene", 0, QApplication::UnicodeUTF8));
        actionReset_Everything->setText(QApplication::translate("MainWindow", "Reset Everything", 0, QApplication::UnicodeUTF8));
        simOptionsBox->setTitle(QApplication::translate("MainWindow", "Simulation Options", 0, QApplication::UnicodeUTF8));
        SimulationBox->setTitle(QApplication::translate("MainWindow", "Simulation Controls", 0, QApplication::UnicodeUTF8));
        startSimulationButton->setText(QApplication::translate("MainWindow", "Start Simulation", 0, QApplication::UnicodeUTF8));
        SimParametersBox->setTitle(QApplication::translate("MainWindow", "Parameters", 0, QApplication::UnicodeUTF8));
        timeStepLabel->setText(QApplication::translate("MainWindow", "Time Step:", 0, QApplication::UnicodeUTF8));
        newtonTolLabel->setText(QApplication::translate("MainWindow", "Newton Tolerance:", 0, QApplication::UnicodeUTF8));
        newtonMaxItersLabel->setText(QApplication::translate("MainWindow", "Newton Max Iters:", 0, QApplication::UnicodeUTF8));
        activeForcesBox->setTitle(QApplication::translate("MainWindow", "Active Forces", 0, QApplication::UnicodeUTF8));
        gravityCheckBox->setText(QApplication::translate("MainWindow", "Gravity", 0, QApplication::UnicodeUTF8));
        floorCheckBox->setText(QApplication::translate("MainWindow", "Floor", 0, QApplication::UnicodeUTF8));
        gravityGLabel->setText(QApplication::translate("MainWindow", "Acceleration:", 0, QApplication::UnicodeUTF8));
        floorStiffness->setText(QApplication::translate("MainWindow", "Stiffness:", 0, QApplication::UnicodeUTF8));
        uiOptionsBox->setTitle(QApplication::translate("MainWindow", "UI Options", 0, QApplication::UnicodeUTF8));
        rigidBodyTypeBox->setTitle(QApplication::translate("MainWindow", "Rigid Body Type", 0, QApplication::UnicodeUTF8));
        sphereButton->setText(QApplication::translate("MainWindow", "Sphere", 0, QApplication::UnicodeUTF8));
        twoByFourButton->setText(QApplication::translate("MainWindow", "Two by Four", 0, QApplication::UnicodeUTF8));
        bunnyButton->setText(QApplication::translate("MainWindow", "Bunny", 0, QApplication::UnicodeUTF8));
        customButton->setText(QApplication::translate("MainWindow", "Custom", 0, QApplication::UnicodeUTF8));
        launchOptionsBox->setTitle(QApplication::translate("MainWindow", "Rigid Body Options", 0, QApplication::UnicodeUTF8));
        launchVelLabel->setText(QApplication::translate("MainWindow", "Launch Velocity:", 0, QApplication::UnicodeUTF8));
        randomOrienatationCheckBox->setText(QApplication::translate("MainWindow", "Random Orientation", 0, QApplication::UnicodeUTF8));
        randomAngularVelCheckBox->setText(QApplication::translate("MainWindow", "Random Angular Vel", 0, QApplication::UnicodeUTF8));
        densityLabel->setText(QApplication::translate("MainWindow", "Density:", 0, QApplication::UnicodeUTF8));
        randomVelMagLabel->setText(QApplication::translate("MainWindow", "Magnitude:", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("MainWindow", "File", 0, QApplication::UnicodeUTF8));
        menuScene->setTitle(QApplication::translate("MainWindow", "Scene", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
