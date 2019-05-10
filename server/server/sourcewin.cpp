//!
//! \file   sourcewin.cpp
//! \author Ulrich Buettemeier
//! \mainclass MainWindow
//!
#include <QMainWindow>
// #include <QLayout>
// #include <QVBoxLayout>

#include "mainwindow.h"
#include "parawin.h"
#include "sourcewin.h"


Sourcewin *sourcewin = NULL;

//!
//! \brief Sourcewin::Sourcewin
//! \param func
//!
Sourcewin::Sourcewin (struct _cvd_func_ *func, MainWindow *main_win, QWidget *parent)
{
    Q_UNUSED (parent);

    cf = func;
    mw = main_win;

    this->setGeometry(350, 10, 700, 150);

    QFont font;
    font.setFamily("Courier");
    font.setFixedPitch(true);
    font.setPointSize(12);

    te = new QPlainTextEdit();
    te->setFont( font );
    te->setLineWrapMode(te->NoWrap);
    highlighter = new Highlighter(te->document());

    VBox = new QVBoxLayout;
    VBox->setMargin(2);
    VBox->setSpacing(0);

    VBox->addWidget (te);
    setLayout( VBox );

    if (cf != NULL) {
        this->setWindowTitle(cf->func_name);
        QFile qf( cf->filename );
        if (qf.exists()) {
            if (qf.open( QIODevice::ReadOnly | QIODevice::Text )) {
                QTextStream in(&qf);
                uint32_t znr = 0;
                while (!in.atEnd()) {
                    QString s = in.readLine();      // Zeile einlesen
                    znr++;
                    if (znr == cf->line_nr) {

                    }
                    if (!in.atEnd()) {
                        te->insertPlainText(QString("%1 %2\n").arg(QString::asprintf("%-4i", znr)).arg(QString(s)));
                    }
                }
            } // if (qf.open( QIODevice::ReadOnly | QIODevice::Text ))
        } // if (qf.exists())
    } else {
        this->setWindowTitle("No source data");
    }    

    QTextCursor cursor(te->document()->findBlockByLineNumber(cf->line_nr-1)); // ln-1 because line number starts from 0
    te->setTextCursor(cursor);
    highlightCurrentLine();         // Cursor-Pos Zeile wird gehighlightet
    te->setReadOnly( true );

    /*
    te->moveCursor (QTextCursor::Start);
    te->ensureCursorVisible();
    */

    this->show();
}

//!
//! \brief Sourcewin::~Sourcewin
//!
Sourcewin::~Sourcewin ()
{
    // printf ("Destruktor Sourcewin\n");
    mw->set_all_source_icon ( false );
    sourcewin = NULL;
}

//!
//! \brief Sourcewin::closeEvent
//! \param bar unused
//!
void Sourcewin::closeEvent(QCloseEvent *bar)
{
    Q_UNUSED (bar);

    // printf ("closeEvent\n");
    delete sourcewin;
}

//!
//! \brief Sourcewin::highlightCurrentLine
//!
void Sourcewin::highlightCurrentLine()
{
    QList<QTextEdit::ExtraSelection> extraSelections;
    if (!te->isReadOnly()) {
        QTextEdit::ExtraSelection selection;
        QColor lineColor = QColor(Qt::yellow).lighter(160);
        selection.format.setBackground(lineColor);
        selection.format.setProperty(QTextFormat::FullWidthSelection, true);
        selection.cursor = te->textCursor();
        selection.cursor.clearSelection();
        extraSelections.append(selection);
    }
    te->setExtraSelections(extraSelections);
}

//!
//! \brief Highlighter::Highlighter
//! \param parent
//!
Highlighter::Highlighter(QTextDocument *parent)
    : QSyntaxHighlighter(parent)
{
    HighlightingRule rule;

    keywordFormat.setForeground(Qt::darkBlue);
    keywordFormat.setFontWeight(QFont::Bold);
    QStringList keywordPatterns;
    keywordPatterns << "\\bchar\\b" << "\\bclass\\b" << "\\bconst\\b"
                    << "\\bdouble\\b" << "\\benum\\b" << "\\bexplicit\\b"
                    << "\\bfriend\\b" << "\\binline\\b" << "\\bint\\b"
                    << "\\blong\\b" << "\\bnamespace\\b" << "\\boperator\\b"
                    << "\\bprivate\\b" << "\\bprotected\\b" << "\\bpublic\\b"
                    << "\\bshort\\b" << "\\bsignals\\b" << "\\bsigned\\b"
                    << "\\bslots\\b" << "\\bstatic\\b" << "\\bstruct\\b"
                    << "\\btemplate\\b" << "\\btypedef\\b" << "\\btypename\\b"
                    << "\\bunion\\b" << "\\bunsigned\\b" << "\\bvirtual\\b"
                    << "\\bvoid\\b" << "\\bvolatile\\b" << "\\bbool\\b"
                    << "\\bCVD\\b";
    foreach (const QString &pattern, keywordPatterns) {
        rule.pattern = QRegularExpression(pattern);
        rule.format = keywordFormat;
        highlightingRules.append(rule);
    }

    classFormat.setFontWeight(QFont::Bold);
    classFormat.setForeground(Qt::darkMagenta);
    rule.pattern = QRegularExpression("\\bQ[A-Za-z]+\\b");
    rule.format = classFormat;
    highlightingRules.append(rule);

    singleLineCommentFormat.setForeground(Qt::red);
    rule.pattern = QRegularExpression("//[^\n]*");
    rule.format = singleLineCommentFormat;
    highlightingRules.append(rule);

    multiLineCommentFormat.setForeground(Qt::red);

    quotationFormat.setForeground(Qt::darkGreen);
    rule.pattern = QRegularExpression("\".*\"");
    rule.format = quotationFormat;
    highlightingRules.append(rule);

    functionFormat.setFontItalic(true);
    functionFormat.setForeground(Qt::blue);
    rule.pattern = QRegularExpression("\\b[A-Za-z0-9_]+(?=\\()");
    rule.format = functionFormat;
    highlightingRules.append(rule);

    commentStartExpression = QRegularExpression("/\\*");
    commentEndExpression = QRegularExpression("\\*/");
}

//!
//! \brief Highlighter::highlightBlock
//! \param text
//!
void Highlighter::highlightBlock(const QString &text)
{
    foreach (const HighlightingRule &rule, highlightingRules) {
        QRegularExpressionMatchIterator matchIterator = rule.pattern.globalMatch(text);
        while (matchIterator.hasNext()) {
            QRegularExpressionMatch match = matchIterator.next();
            setFormat(match.capturedStart(), match.capturedLength(), rule.format);
        }
    }
    setCurrentBlockState(0);
    int startIndex = 0;
    if (previousBlockState() != 1)
        startIndex = text.indexOf(commentStartExpression);

    while (startIndex >= 0) {
        QRegularExpressionMatch match = commentEndExpression.match(text, startIndex);
        int endIndex = match.capturedStart();
        int commentLength = 0;
        if (endIndex == -1) {
            setCurrentBlockState(1);
            commentLength = text.length() - startIndex;
        } else {
            commentLength = endIndex - startIndex
                            + match.capturedLength();
        }
        setFormat(startIndex, commentLength, multiLineCommentFormat);
        startIndex = text.indexOf(commentStartExpression, startIndex + commentLength);
    }
}

