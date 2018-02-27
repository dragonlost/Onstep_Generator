#include "configurator.h"
#include "ui_configurator.h"

configurator::configurator(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::configurator)
{
    ui->setupUi(this);
}

configurator::~configurator()
{
    delete ui;
}
