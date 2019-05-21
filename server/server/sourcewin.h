#ifndef SOURCEWIN_H
#define SOURCEWIN_H

#include <QWidget>
#include <QTextEdit>
#include <QPlainTextEdit>
#include <QVBoxLayout>
#include <QSyntaxHighlighter>
#include <QTextCharFormat>
#include <QRegularExpression>

#include "mainwindow.h"
#include "opencvd_types.hpp"
// #include "cvd_editor.h"

QT_BEGIN_NAMESPACE
class QTextDocument;
QT_END_NAMESPACE

//!
//! \brief The Highlighter class
//!
class Highlighter : public QSyntaxHighlighter
{
    Q_OBJECT

public:
    Highlighter(QTextDocument *parent = 0);

protected:
    void highlightBlock(const QString &text) override;

private:
    struct HighlightingRule
    {
        QRegularExpression pattern;
        QTextCharFormat format;
    };
    QVector<HighlightingRule> highlightingRules;

    QRegularExpression commentStartExpression;
    QRegularExpression commentEndExpression;

    QTextCharFormat keywordFormat;
    QTextCharFormat classFormat;
    QTextCharFormat singleLineCommentFormat;
    QTextCharFormat multiLineCommentFormat;
    QTextCharFormat quotationFormat;
    QTextCharFormat functionFormat;
};

//!
//! \brief The Sourcewin class
//!
class Sourcewin : public QWidget
{
    Q_OBJECT
public:
    Sourcewin (struct _cvd_func_ *func, MainWindow *main_win, QWidget *parent = nullptr);
    ~Sourcewin ();

public:
    struct _cvd_func_ *cf = NULL;
    QPlainTextEdit *te;
    Highlighter *highlighter;

private:
    void closeEvent(QCloseEvent *bar);    
    void keyPressEvent(QKeyEvent *event);
    void highlightCurrentLine( void );

private:
    MainWindow *mw = NULL;    
    QVBoxLayout *VBox = NULL;
};

extern Sourcewin *sourcewin;

#endif // SOURCEWIN_H
