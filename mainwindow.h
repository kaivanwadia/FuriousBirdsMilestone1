#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

class Controller;
struct SimParameters;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(Controller &cont, int fps, QWidget *parent = 0);
    ~MainWindow();   

public slots:
    void setUIFromParameters(const SimParameters &params);
    void setScore(const int score);

private slots:
    void updateGL();

    void on_actionExit_triggered();

    void on_actionReset_Everything_triggered();

    void on_actionReset_triggered();

    void on_startSimulationButton_clicked();

    void on_timeStepEdit_editingFinished();

    void on_newtonTolEdit_editingFinished();

    void on_newtonMaxItersEdit_editingFinished();

    void on_gravityCheckBox_clicked();

    void on_gameModeCheckBox_clicked();

    void on_floorCheckBox_clicked();

    void on_gravityGEdit_editingFinished();

    void on_floorStiffnessEdit_editingFinished();

    void on_sphereButton_clicked();

    void on_twoByFourButton_clicked();

    void on_bunnyButton_clicked();

    void on_customButton_clicked();

    void on_launchVelEdit_editingFinished();

    void on_randomOrienatationCheckBox_clicked();

    void on_randomAngularVelCheckBox_clicked();

    void on_randomVelMagEdit_editingFinished();

    void on_densityEdit_editingFinished();

private:
    Controller &cont_;
    Ui::MainWindow *ui;
    bool simRunning_;
    QTimer renderTimer_;

    void setParametersFromUI();
};

#endif // MAINWINDOW_H
