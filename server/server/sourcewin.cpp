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


Sourcewin *sourcewin = nullptr;    // global pointer

//!
//! \brief Sourcewin::Sourcewin
//! \param func
//! \todo parameter main_win enventuell entfernen. Es gibt den blobal pointer MainWindow *glob_mw
//!
Sourcewin::Sourcewin (struct _cvd_func_ *func, MainWindow *main_win, QWidget *parent)
{
    Q_UNUSED (parent);

    cf = func;
    mw = main_win;

    this->setGeometry(glob_mw->source_win_pos.x(),
                      glob_mw->source_win_pos.y(),
                      glob_mw->source_win_pos.width(),
                      glob_mw->source_win_pos.height());

    QFont font;
    font.setFamily("Courier");
    font.setFixedPitch(true);
    font.setPointSize(12);

    mod_source = new QPushButton ("insert parameter comment");    
    //! @todo the button width is not usefull
    // mod_source->setMaximumWidth( 150 );
    // mod_source->setMinimumWidth( 150 );
    mod_source->setParent( this );
    connect (mod_source, SIGNAL(clicked(bool)), this, SLOT(source_mod()));

    close_win = new QPushButton ("Close");
    close_win->setFixedWidth( 150 );
    close_win->setParent( this );
    connect (close_win, SIGNAL(clicked(bool)), this, SLOT(close_win_pushed()));

    HBox = new QHBoxLayout;

    HBox->setMargin(2);
    HBox->setSpacing(0);
    HBox->addWidget( mod_source );
    HBox->addWidget( close_win );    
    HBox->insertSpacing(3, 200);

    HGroup = new QGroupBox ();
    HGroup->setStyleSheet("QGroupBox{padding-top:15px; margin-top:-15px}");     // HGroup without title
    HGroup->setLayout( HBox );

    te = new QPlainTextEdit();
    te->setFont( font );
    te->setLineWrapMode(te->NoWrap);
    highlighter = new Highlighter(te->document());

    VBox = new QVBoxLayout;
    VBox->setMargin(2);
    VBox->setSpacing(0);

    VBox->addWidget ( te );         // Editor
    VBox->addWidget( HGroup );      // Buttons

    setLayout( VBox );

    read_source();

    this->show();
}

//!
//! \brief Sourcewin::read_source
//!
void Sourcewin::read_source ()
{
    te->clear();

    if (cf != nullptr) {
        this->setWindowTitle(cf->func_name);
        QFile qf( cf->filename );
        if (qf.exists()) {
            if (qf.open( QIODevice::ReadOnly | QIODevice::Text )) {
                QTextStream in(&qf);
                uint32_t znr = 0;
                while (!in.atEnd()) {
                    QString s = in.readLine();      // Zeile einlesen
                    znr++;
                    if (!in.atEnd()) {
                        char a[16];
                        sprintf (a, "%-4i", znr);   // Zeilen-Nr to string
                        te->insertPlainText(QString("%1 %2\n").arg(QString(a)).arg(QString(s)));
                    }
                }
                qf.close();
            } // if (qf.open( QIODevice::ReadOnly | QIODevice::Text ))
        } // if (qf.exists())
    } else {
        this->setWindowTitle("No source data");
    }

    QTextCursor cursor(te->document()->findBlockByLineNumber(cf->line_nr-1)); // ln-1 because line number starts from 0
    te->setTextCursor(cursor);
    highlightCurrentLine();         // Cursor-Pos Zeile wird gehighlightet
    te->setReadOnly( true );

    te->show();
}

//!
//! \brief Sourcewin::modify_source
//! \param z_nr
//!
void Sourcewin::modify_source (uint32_t z_nr)
{
    std::vector <QString> vs = {};

    if (cf != nullptr) {
        this->setWindowTitle(cf->func_name);
        QFile qf( cf->filename );
        if (qf.exists()) {
            if (qf.open( QIODevice::ReadOnly | QIODevice::Text )) {
                QTextStream in(&qf);
                uint32_t znr = 0;
                while (!in.atEnd()) {
                    vs.push_back(in.readLine());    // Zeile einlesen
                    znr++;
                    if (znr == z_nr)                // Kommentar einfuegen
                        vs.push_back( mw->build_source_line_comment( cf ));

                }
                qf.close();

                QFile qf( cf->filename );
                if (qf.open( QIODevice::WriteOnly | QIODevice::Text )) {

                    for (uint32_t i=0; i < (uint32_t)vs.size(); i++) {
                        qf.write(vs[i].toStdString().c_str(), vs[i].length());      // write Zeile
                        qf.write("\n");                                             // Line Feed
                    }
                    qf.close();
                } else {
                    QMessageBox::warning(this,
                                         "File ERROR",
                                         "File open error.\nPossibly no write permissions.",
                                         QMessageBox::Ok);
                }
            } // if (qf.open( QIODevice::ReadOnly | QIODevice::Text ))
        } // if (qf.exists())
    } else {
        this->setWindowTitle("No source data");
    }

}

//!
//! \brief Sourcewin::~Sourcewin
//!
Sourcewin::~Sourcewin ()
{
    glob_mw->source_win_pos = this->geometry();     // get rectangle

    mw->set_all_source_icon ( false );
    sourcewin = nullptr;
}

//!
//! \brief Sourcewin::source_mod
//!
void Sourcewin::source_mod ()
{
    struct _cvd_func_ *foo = mw->first_func;

    modify_source (cf->line_nr);

    while (foo != nullptr) {
        if (foo->line_nr > cf->line_nr)
            foo->line_nr++;
        foo = foo->next;
    }

    mw->refresh_source_win( cf );
}

//!
//! \brief Sourcewin::close_win_pushed
//!
void Sourcewin::close_win_pushed ()
{
    close();
}

//!
//! \brief Sourcewin::keyPressEvent
//! \param event
//!
void Sourcewin::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_Escape)
        close();
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

