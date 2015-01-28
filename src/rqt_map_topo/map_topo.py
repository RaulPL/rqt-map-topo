__author__ = 'Raul Peralta-Lozada'
__email__ = 'raulpl25@gmail.com'


import argparse

from qt_gui.plugin import Plugin
from .map_widget import MapTopoWidget

class MapTopo(Plugin):
    """
    Subclass of Plugin to save semantic locations on a grid map.
    """
    def __init__(self, context):
        super(MapTopo, self).__init__(context)
        self.setObjectName('MapGuiPlugin')

        args = self._parse_args(context.argv())
        # args, unknowns = parser.parse_known_args(context.argv())

        # Create QWidget
        self._widget = MapTopoWidget(context)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

    def _parse_args(self, argv):
        parser = argparse.ArgumentParser(prog='rqt_map_topo', add_help=False)
        MapTopo.add_arguments(parser)
        return parser.parse_args(argv)

    @staticmethod
    def add_arguments(parser):
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")


    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog