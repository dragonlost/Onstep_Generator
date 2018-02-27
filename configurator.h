#ifndef CONFIGURATOR_H
#define CONFIGURATOR_H

#include <QMainWindow>

namespace Ui {
class configurator;
}

class configurator : public QMainWindow
{
    Q_OBJECT

public:
    explicit configurator(QWidget *parent = 0);
    ~configurator();

private:
    Ui::configurator *ui;
};

#endif // CONFIGURATOR_H
