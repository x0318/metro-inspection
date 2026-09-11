"""Qt WebEngine report output shared by both desktop entry points."""

from datetime import datetime
from pathlib import Path

from PyQt5.QtCore import QMarginsF, QStandardPaths, QTimer
from PyQt5.QtGui import QPageLayout, QPageSize
from PyQt5.QtPrintSupport import QPrintDialog, QPrinter
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtWidgets import QDialog, QFileDialog, QMessageBox


class ReportView(QWebEngineView):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.output_busy = False
        self.printer = None
        self.page().printRequested.connect(self.request_output)
        self.page().pdfPrintingFinished.connect(self.pdf_finished)

    def request_output(self):
        if not self.output_busy:
            self.output_busy = True
            QTimer.singleShot(0, self.choose_output)

    def choose_output(self):
        dialog = QMessageBox(self)
        dialog.setWindowTitle("打印 / 导出 PDF")
        dialog.setText("选择巡检报告的输出方式")
        pdf = dialog.addButton("导出 PDF", QMessageBox.AcceptRole)
        paper = dialog.addButton("打印", QMessageBox.ActionRole)
        dialog.addButton("取消", QMessageBox.RejectRole)
        dialog.setDefaultButton(pdf)
        dialog.exec_()
        try:
            if dialog.clickedButton() is pdf:
                self.export_pdf()
            elif dialog.clickedButton() is paper:
                self.print_report()
            else:
                self.output_busy = False
        except (OSError, RuntimeError, ValueError) as error:
            self.output_busy = False
            self.printer = None
            QMessageBox.warning(self, "报告输出失败", str(error))

    @staticmethod
    def report_layout():
        return QPageLayout(QPageSize(QPageSize.A4), QPageLayout.Landscape,
                           QMarginsF(12, 12, 12, 12), QPageLayout.Millimeter)

    def export_pdf(self):
        folder = QStandardPaths.writableLocation(QStandardPaths.DocumentsLocation)
        name = f"巡检报告-{datetime.now():%Y%m%d-%H%M%S}.pdf"
        path, _ = QFileDialog.getSaveFileName(
            self, "导出巡检报告", str(Path(folder or str(Path.home())) / name), "PDF (*.pdf)"
        )
        if not path:
            self.output_busy = False
            return
        self.output_busy = True
        self.page().printToPdf(path, self.report_layout())

    def pdf_finished(self, path, success):
        self.output_busy = False
        if success:
            QMessageBox.information(self, "报告已导出", f"已保存到：\n{path}")
        else:
            QMessageBox.warning(self, "导出失败", "无法生成 PDF，请检查保存目录是否可写后重试。")

    def print_report(self):
        # WebEngine prints asynchronously; the printer must outlive the dialog.
        self.printer = QPrinter(QPrinter.HighResolution)
        self.printer.setResolution(300)
        self.printer.setPageLayout(self.report_layout())
        dialog = QPrintDialog(self.printer, self)
        dialog.setWindowTitle("打印巡检报告")
        if dialog.exec_() != QDialog.Accepted:
            self.printer = None
            self.output_busy = False
            return
        self.output_busy = True
        self.page().print(self.printer, self.print_finished)

    def print_finished(self, success):
        self.printer = None
        self.output_busy = False
        if not success:
            QMessageBox.warning(self, "打印失败", "报告未能提交打印，请检查打印机，或选择导出 PDF。")
