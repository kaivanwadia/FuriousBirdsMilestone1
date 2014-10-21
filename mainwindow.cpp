#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "simparameters.h"
#include "controller.h"
#include <iostream>

MainWindow::MainWindow(Controller &cont, int fps, QWidget *parent) :
    QMainWindow(parent),
    cont_(cont),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->GLWidget->setController(&cont);
    simRunning_ = false;
    connect(&renderTimer_, SIGNAL(timeout()), this, SLOT(updateGL()));
    renderTimer_.start(1000/fps);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_actionExit_triggered()
{
    close();
}

void MainWindow::setParametersFromUI()
{
    SimParameters params;

    params.simRunning = simRunning_;

    params.timeStep = ui->timeStepEdit->text().toDouble();
    params.NewtonTolerance = ui->newtonTolEdit->text().toDouble();
    params.NewtonMaxIters = ui->newtonMaxItersEdit->text().toInt();

    params.activeForces = 0;
    if(ui->gravityCheckBox->isChecked())
        params.activeForces |= SimParameters::F_GRAVITY;
    if(ui->floorCheckBox->isChecked())
        params.activeForces |= SimParameters::F_FLOOR;

    params.gravityG = ui->gravityGEdit->text().toDouble();
    params.floorStiffness = ui->floorStiffnessEdit->text().toDouble();

    params.bodyDensity = ui->densityEdit->text().toDouble();

    if(ui->sphereButton->isChecked())
        params.launchBody = SimParameters::R_SPHERE;
    else if(ui->twoByFourButton->isChecked())
        params.launchBody = SimParameters::R_2BY4;
    else if(ui->bunnyButton->isChecked())
        params.launchBody = SimParameters::R_BUNNY;
    else if(ui->customButton->isChecked())
        params.launchBody = SimParameters::R_CUSTOM;

    params.launchVel = ui->launchVelEdit->text().toDouble();
    params.randomLaunchAngVel = ui->randomAngularVelCheckBox->isChecked();
    params.randomLaunchOrientation = ui->randomOrienatationCheckBox->isChecked();
    params.randomLaunchVelMagnitude = ui->randomVelMagEdit->text().toDouble();
    params.gameMode = ui->gameModeCheckBox->isChecked();

    setUIFromParameters(params);
    QMetaObject::invokeMethod(&cont_, "updateParameters", Q_ARG(SimParameters, params));
}

void MainWindow::setUIFromParameters(const SimParameters &params)
{
    if(params.simRunning)
    {
        ui->startSimulationButton->setText(QString("Pause Simulation"));
        simRunning_ = true;
    }
    else
    {
        ui->startSimulationButton->setText(QString("Start Simulation"));
        simRunning_ = false;
    }

    ui->timeStepEdit->setText(QString::number(params.timeStep));
    ui->newtonTolEdit->setText(QString::number(params.NewtonTolerance));
    ui->newtonMaxItersEdit->setText(QString::number(params.NewtonMaxIters));

    ui->gravityCheckBox->setChecked(params.activeForces & SimParameters::F_GRAVITY);
    ui->floorCheckBox->setChecked(params.activeForces & SimParameters::F_FLOOR);
    ui->gameModeCheckBox->setChecked(params.gameMode);

    ui->gravityGEdit->setText(QString::number(params.gravityG));       
    ui->floorStiffnessEdit->setText(QString::number(params.floorStiffness));

    ui->densityEdit->setText(QString::number(params.bodyDensity));

    switch(params.launchBody)
    {
    case SimParameters::R_SPHERE:
        ui->sphereButton->setChecked(true);
        break;
    case SimParameters::R_2BY4:
        ui->twoByFourButton->setChecked(true);
        break;
    case SimParameters::R_BUNNY:
        ui->bunnyButton->setChecked(true);
        break;
    case SimParameters::R_CUSTOM:
        ui->customButton->setChecked(true);
        break;
    }

    ui->launchVelEdit->setText(QString::number(params.launchVel));
    ui->randomOrienatationCheckBox->setChecked(params.randomLaunchOrientation);
    ui->randomAngularVelCheckBox->setChecked(params.randomLaunchAngVel);
    ui->randomVelMagEdit->setText(QString::number(params.randomLaunchVelMagnitude));

    ui->gameModeCheckBox->setChecked(params.gameMode);
}

void MainWindow::setScore(const int score)
{
    std::ostringstream highScore;
    highScore <<"High Score: "<<score;
    QString hScore(highScore.str().c_str());
    ui->scoreLabel->setText(hScore);
}

void MainWindow::updateGL()
{
    ui->GLWidget->tick();
    ui->GLWidget->update();
}

void MainWindow::on_actionReset_Everything_triggered()
{
    QMetaObject::invokeMethod(&cont_, "reset");
}

void MainWindow::on_actionReset_triggered()
{
    QMetaObject::invokeMethod(&cont_, "clearScene");
}

void MainWindow::on_startSimulationButton_clicked()
{
    simRunning_ = !simRunning_;
    setParametersFromUI();
}

void MainWindow::on_timeStepEdit_editingFinished()
{
    setParametersFromUI();
}

void MainWindow::on_newtonTolEdit_editingFinished()
{
    setParametersFromUI();
}

void MainWindow::on_newtonMaxItersEdit_editingFinished()
{
    setParametersFromUI();
}

void MainWindow::on_gravityCheckBox_clicked()
{
    setParametersFromUI();
}

void MainWindow::on_gameModeCheckBox_clicked()
{
    if(simRunning_)
    {
        on_startSimulationButton_clicked();
    }
    QMetaObject::invokeMethod(&cont_, "clearScene");
    if(ui->gameModeCheckBox->isChecked())
    {
        QMetaObject::invokeMethod(&cont_, "setupGame");
    }
    setParametersFromUI();
}

void MainWindow::on_floorCheckBox_clicked()
{
    setParametersFromUI();
}

void MainWindow::on_gravityGEdit_editingFinished()
{
    setParametersFromUI();
}

void MainWindow::on_floorStiffnessEdit_editingFinished()
{
    setParametersFromUI();
}

void MainWindow::on_sphereButton_clicked()
{
    setParametersFromUI();
}

void MainWindow::on_twoByFourButton_clicked()
{
    setParametersFromUI();
}

void MainWindow::on_bunnyButton_clicked()
{
    setParametersFromUI();
}

void MainWindow::on_customButton_clicked()
{
    setParametersFromUI();
}

void MainWindow::on_launchVelEdit_editingFinished()
{
    setParametersFromUI();
}

void MainWindow::on_randomOrienatationCheckBox_clicked()
{
    setParametersFromUI();
}

void MainWindow::on_randomAngularVelCheckBox_clicked()
{
    setParametersFromUI();
}

void MainWindow::on_randomVelMagEdit_editingFinished()
{
    setParametersFromUI();
}

void MainWindow::on_densityEdit_editingFinished()
{
    setParametersFromUI();
}
