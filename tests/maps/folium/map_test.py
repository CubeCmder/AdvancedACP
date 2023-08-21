import sys
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout
from PyQt6.QtWebEngineWidgets import QWebEngineView
import folium

class MapWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.layout = QVBoxLayout(self)
        self.setLayout(self.layout)

        self.browser = QWebEngineView()
        self.layout.addWidget(self.browser)

    def load_map(self, lat, lon):
        m = folium.Map(location=[lat, lon], zoom_start=12)
        data = m._repr_html_()
        self.browser.setHtml(data)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    widget = MapWidget()
    widget.load_map(45.51750517740884, -73.78426214537873)  # Pass the desired coordinates here
    widget.show()
    sys.exit(app.exec())
