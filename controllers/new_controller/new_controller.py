from PySide6.QtWidgets import (
    QApplication,
    QGraphicsScene,
    QMainWindow,
)
from PySide6.QtGui import QColor, QPolygonF
from PySide6.QtCore import (
        QPoint,
        QThread,
        QCoreApplication,
)
from webots import WebotsWorker

from hexapod_controller_ui import Ui_HexapodController


WEBOTS_WORKER = None


class ApplicationMainWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = Ui_HexapodController()
        self.ui.setupUi(self)

        self.setup()

    def setup(self):
        self.leg_spacing_scene = self.draw_leg_spacing_scene(40)
        self.ui.leg_spacing_view.setScene(self.leg_spacing_scene)
        self.height_scene = self.draw_height_scene(20)
        self.ui.height_view.setScene(self.height_scene)
        self.ui.leg_spacing_slider.sliderMoved.connect(self.update_leg_spacing)
        self.ui.height_slider.sliderMoved.connect(self.update_height)
        self.ui.vdir_dial.sliderMoved.connect(self.update_vdir)
        self.ui.vval_slider.sliderMoved.connect(self.update_vval)
        self.ui.omega_dial.sliderMoved.connect(self.update_omega)
        self.ui.yaw_dial.sliderMoved.connect(self.update_yaw)
        self.ui.pitch_dial.sliderMoved.connect(self.update_pitch)
        self.ui.roll_dial.sliderMoved.connect(self.update_roll)
        self.ui.step_height_slider.sliderMoved.connect(self.update_step_height)

    def update_leg_spacing(self, spacing):
        self.leg_spacing_scene = self.draw_leg_spacing_scene((spacing-40)//2)
        self.ui.leg_spacing_view.setScene(self.leg_spacing_scene)
        WEBOTS_WORKER.leg_spacing_signal.emit(spacing)

    def update_height(self, height):
        self.height_scene = self.draw_height_scene(height//5)
        self.ui.height_view.setScene(self.height_scene)
        WEBOTS_WORKER.height_signal.emit(height)

    def update_vdir(self, vdir):
        WEBOTS_WORKER.vdir_signal.emit(vdir)

    def update_vval(self, vval):
        WEBOTS_WORKER.vval_signal.emit(vval)

    def update_omega(self, omega):
        WEBOTS_WORKER.omega_signal.emit(omega)

    def update_yaw(self, yaw):
        WEBOTS_WORKER.yaw_signal.emit(yaw)

    def update_pitch(self, pitch):
        WEBOTS_WORKER.pitch_signal.emit(pitch)

    def update_roll(self, roll):
        WEBOTS_WORKER.roll_signal.emit(roll)

    def update_step_height(self, step_height):
        WEBOTS_WORKER.step_height_signal.emit(step_height)

    def draw_leg_spacing_scene(self, leg_len):
        scene = QGraphicsScene(self)

        corpus = QPolygonF([
            QPoint(0, 0),
            QPoint(40, 0),
            QPoint(40, 60),
            QPoint(0, 60),
        ])
        leg1 = QPolygonF([
            QPoint(-leg_len, 0),
            QPoint(0, 0),
            QPoint(0, 5),
            QPoint(-leg_len, 5),
        ])
        leg2 = QPolygonF([
            QPoint(-leg_len, 28),
            QPoint(0, 28),
            QPoint(0, 33),
            QPoint(-leg_len, 33),
        ])
        leg3 = QPolygonF([
            QPoint(-leg_len, 55),
            QPoint(0, 55),
            QPoint(0, 60),
            QPoint(-leg_len, 60),
        ])
        leg4 = QPolygonF([
            QPoint(40, 0),
            QPoint(40+leg_len, 0),
            QPoint(40+leg_len, 5),
            QPoint(40, 5),
        ])
        leg5 = QPolygonF([
            QPoint(40, 28),
            QPoint(40+leg_len, 28),
            QPoint(40+leg_len, 33),
            QPoint(40, 33),
        ])
        leg6 = QPolygonF([
            QPoint(40, 55),
            QPoint(40+leg_len, 55),
            QPoint(40+leg_len, 60),
            QPoint(40, 60),
        ])

        scene.addPolygon(corpus,
                         QColor(255, 0, 0, 255),
                         QColor(255, 0, 0, 255))
        scene.addPolygon(leg1,
                         QColor(0, 0, 0, 255),
                         QColor(0, 0, 0, 255))
        scene.addPolygon(leg2,
                         QColor(0, 0, 0, 255),
                         QColor(0, 0, 0, 255))
        scene.addPolygon(leg3,
                         QColor(0, 0, 0, 255),
                         QColor(0, 0, 0, 255))
        scene.addPolygon(leg4,
                         QColor(0, 0, 0, 255),
                         QColor(0, 0, 0, 255))
        scene.addPolygon(leg5,
                         QColor(0, 0, 0, 255),
                         QColor(0, 0, 0, 255))
        scene.addPolygon(leg6,
                         QColor(0, 0, 0, 255),
                         QColor(0, 0, 0, 255))

        return scene

    def draw_height_scene(self, height):
        scene = QGraphicsScene(self)

        corpus = QPolygonF([
            QPoint(0, -3),
            QPoint(40, -3),
            QPoint(40, 7),
            QPoint(0, 7),
        ])
        leglt = QPolygonF([
            QPoint(-40, 0),
            QPoint(0, 0),
            QPoint(0, 5),
            QPoint(-40, 5),
        ])
        legrt = QPolygonF([
            QPoint(40, 0),
            QPoint(80, 0),
            QPoint(80, 5),
            QPoint(40, 5),
        ])
        leglb = QPolygonF([
            QPoint(-40, 5),
            QPoint(-35, 5),
            QPoint(-35, 5+height),
            QPoint(-40, 5+height),
        ])
        legrb = QPolygonF([
            QPoint(75, 5),
            QPoint(80, 5),
            QPoint(80, 5+height),
            QPoint(75, 5+height),
        ])

        scene.addPolygon(corpus,
                         QColor(255, 0, 0, 255),
                         QColor(255, 0, 0, 255))
        scene.addPolygon(leglt,
                         QColor(0, 0, 0, 255),
                         QColor(0, 0, 0, 255))
        scene.addPolygon(leglb,
                         QColor(0, 0, 0, 255),
                         QColor(0, 0, 0, 255))
        scene.addPolygon(legrt,
                         QColor(0, 0, 0, 255),
                         QColor(0, 0, 0, 255))
        scene.addPolygon(legrb,
                         QColor(0, 0, 0, 255),
                         QColor(0, 0, 0, 255))

        return scene


if __name__ == "__main__":
    thread = QThread()
    WEBOTS_WORKER = WebotsWorker()
    WEBOTS_WORKER.moveToThread(thread)
    thread.started.connect(WEBOTS_WORKER.run)
    WEBOTS_WORKER.finished.connect(thread.quit)
    WEBOTS_WORKER.finished.connect(WEBOTS_WORKER.deleteLater)
    thread.finished.connect(thread.deleteLater)
    thread.start()

    app = QApplication()
    window = ApplicationMainWindow()
    window.show()
    app.exec()

    # Clean exit
    WEBOTS_WORKER.stop.emit()
    while not thread.isFinished():
        QCoreApplication.processEvents()
