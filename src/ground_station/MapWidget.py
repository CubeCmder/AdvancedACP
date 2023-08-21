import os

import folium
import requests
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QSizePolicy


class MapWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.layout = QVBoxLayout(self)
        self.setLayout(self.layout)

        self.browser = QWebEngineView()
        self.browser.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.browser.setContentsMargins(0, 0, 0, 0)

        self.polyline = None
        self.layout.addWidget(self.browser)

    def update_polyline(self, coordinates):
        #coordinates.append([45.51845758574176, -73.78473833271342])
        if self.polyline is not None:
            # Update the coordinates of the existing polyline
            self.polyline.locations = coordinates

            # Optionally, update the map's view to focus on the updated polyline's path
            #bounds = self.polyline.get_bounds()
            #self.map.fit_bounds(bounds)

            # Update the map in the output file (if it was saved to a file)
            self.browser.setHtml(self.map._repr_html_())
        else:
            # Add the initial polyline to the map
            self.polyline = folium.PolyLine(locations=coordinates, color='red')
            self.polyline.add_to(self.map)
            self.browser.setHtml(self.map._repr_html_())

    def update_polyline_v2(self, lat, long):
        #coordinates.append([45.51845758574176, -73.78473833271342])

        if self.polyline is not None:
            # Update the coordinates of the existing polyline

            coordinates = self.polyline.locations
            coordinates.append([lat, long])
            # Optionally, update the map's view to focus on the updated polyline's path
            #bounds = self.polyline.get_bounds()
            #self.map.fit_bounds(bounds)

            # Update the map in the output file (if it was saved to a file)
            self.browser.setHtml(self.map._repr_html_())
        else:
            # Add the initial polyline to the map
            self.polyline = folium.PolyLine(locations=[[lat,long]], color='red')
            self.polyline.add_to(self.map)
            self.browser.setHtml(self.map._repr_html_())

    def add_marker(self, latitude, longitude):
        js = Template(
            """
        L.marker([{{latitude}}, {{longitude}}] )
            .addTo({{map}});
        L.circleMarker(
            [{{latitude}}, {{longitude}}], {
                "bubblingMouseEvents": true,
                "color": "#3388ff",
                "dashArray": null,
                "dashOffset": null,
                "fill": false,
                "fillColor": "#3388ff",
                "fillOpacity": 0.2,
                "fillRule": "evenodd",
                "lineCap": "round",
                "lineJoin": "round",
                "opacity": 1.0,
                "radius": 2,
                "stroke": true,
                "weight": 5
            }
        ).addTo({{map}});
        """

        ).render(map=self.map_widget.map.get_name(), latitude=latitude, longitude=longitude)

        self.map_widget.browser.page().runJavaScript(js)

    def load_map(self, lat, lon):
        self.map = folium.Map(location=[lat, lon], zoom_start=16)
        data = self.map._repr_html_()
        self.browser.setHtml(data)

    def save_offline_map(self, save_path):
        # Save the map as an HTML file
        self.map.save(save_path)

        # Get the tile URLs from the map HTML
        with open(save_path, 'r') as file:
            html_content = file.read()

        tile_urls = []
        start_index = 0

        while True:
            start_index = html_content.find('href="', start_index) + len('href="')
            if start_index == -1:
                break
            end_index = html_content.find('.png"', start_index) + len('.png"')
            tile_url = html_content[start_index:end_index]
            tile_urls.append(tile_url)

        # Create the directory for storing the tile images
        tile_dir = os.path.join(os.path.dirname(save_path), 'tiles')
        os.makedirs(tile_dir, exist_ok=True)

        # Download the tile images
        for tile_url in tile_urls:
            tile_path = os.path.join(tile_dir, tile_url)
            if not os.path.exists(tile_path):
                response = requests.get(tile_url)
                with open(tile_path, 'wb') as file:
                    file.write(response.content)

            # Modify the HTML to reference the local tile images
            html_content = html_content.replace(tile_url, 'tiles/' + tile_url)

        # Save the modified HTML file
        with open(save_path, 'w') as file:
            file.write(html_content)