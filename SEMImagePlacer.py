# -*- coding: utf-8 -*-

"""
/***************************************************************************
 SEMImagePlacer
                                 A QGIS plugin
 This plugin places SEM images by their own coordinates
                              -------------------
        begin                : 2015-03-03
        git sha              : $Format:%H$
        copyright            : (C) 2015 by Kouki Kitajima
        email                : saburo@geology.wisc.edu
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
"""
from PyQt4.QtCore import QSettings, QTranslator, qVersion, QCoreApplication, \
    QObject, SIGNAL, Qt
from PyQt4.QtGui import QAction, QIcon, QColor, QFileDialog
from qgis.core import QgsGeometry, QgsPoint, QgsRasterLayer
from qgis.core import QgsVectorLayer, QgsFeature
from qgis.core import QgsVectorFileWriter, QgsMapLayerRegistry
from qgis.core import QgsCoordinateReferenceSystem
from qgis.gui import QgsMapToolEmitPoint, QgsGenericProjectionSelector
# Initialize Qt resources from file resources.py
import resources_rc
# Import the code for the dialog
from SEMImagePlacer_dialog import SEMImagePlacerDialog

import os.path
import os
import re
import subprocess
from math import cos, sin, atan2, radians, degrees


class SEMImagePlacer:
    """QGIS Plugin Implementation."""

    def __init__(self, iface):
        """Constructor.

        :param iface: An interface instance that will be passed to this class
            which provides the hook by which you can manipulate the QGIS
            application at run time.
        :type iface: QgsInterface
        """
        # Save reference to the QGIS interface
        self.iface = iface
        # initialize plugin directory
        self.plugin_dir = os.path.dirname(__file__)
        # initialize locale
        locale = QSettings().value('locale/userLocale')[0:2]
        locale_path = os.path.join(
            self.plugin_dir,
            'i18n',
            'SEMImagePlacer_{}.qm'.format(locale))

        if os.path.exists(locale_path):
            self.translator = QTranslator()
            self.translator.load(locale_path)

            if qVersion() > '4.3.3':
                QCoreApplication.installTranslator(self.translator)

        # Create the dialog (after translation) and keep reference
        self.dlg = SEMImagePlacerDialog()
        # Declare instance attributes
        self.actions = []
        self.menu = self.tr(u'&SEM Image Placer')
        # TODO: We are going to let the user set this up in a future iteration
        self.toolbar = self.iface.addToolBar(u'SEMImagePlacer')
        self.toolbar.setObjectName(u'SEMImagePlacer')

        self.canvas = self.iface.mapCanvas()
        self.clickTool = QgsMapToolEmitPoint(self.canvas)
        self.gdalwarp = os.path.join(QSettings().value('GdalTools/gdalPath'),
                                     'gdalwarp')
        self.crs = "+proj=longlat +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs"
        self.selectingRefFlag = 0
        self.imgData = []
        self.layers = []
        self.referencePoints = {
            '1': {'from': [None, None], 'to': [None, None]},
            '2': {'from': [None, None], 'to': [None, None]}}
        self.connects()
        self.crsId = None
        self.crsType = 0

    # noinspection PyMethodMayBeStatic
    def tr(self, message):
        """Get the translation for a string using Qt translation API.

        We implement this ourselves since we do not inherit QObject.

        :param message: String for translation.
        :type message: str, QString

        :returns: Translated version of message.
        :rtype: QString
        """
        # noinspection PyTypeChecker,PyArgumentList,PyCallByClass
        return QCoreApplication.translate('SEMImagePlacer', message)

    def add_action(
            self,
            icon_path,
            text,
            callback,
            enabled_flag=True,
            add_to_menu=True,
            add_to_toolbar=True,
            status_tip=None,
            whats_this=None,
            parent=None):
        """Add a toolbar icon to the toolbar.

        :param icon_path: Path to the icon for this action. Can be a resource
            path (e.g. ':/plugins/foo/bar.png') or a normal file system path.
        :type icon_path: str

        :param text: Text that should be shown in menu items for this action.
        :type text: str

        :param callback: Function to be called when the action is triggered.
        :type callback: function

        :param enabled_flag: A flag indicating if the action should be enabled
            by default. Defaults to True.
        :type enabled_flag: bool

        :param add_to_menu: Flag indicating whether the action should also
            be added to the menu. Defaults to True.
        :type add_to_menu: bool

        :param add_to_toolbar: Flag indicating whether the action should also
            be added to the toolbar. Defaults to True.
        :type add_to_toolbar: bool

        :param status_tip: Optional text to show in a popup when mouse pointer
            hovers over the action.
        :type status_tip: str

        :param parent: Parent widget for the new action. Defaults None.
        :type parent: QWidget

        :param whats_this: Optional text to show in the status bar when the
            mouse pointer hovers over the action.

        :returns: The action that was created. Note that the action is also
            added to self.actions list.
        :rtype: QAction
        """

        icon = QIcon(icon_path)
        action = QAction(icon, text, parent)
        action.triggered.connect(callback)
        action.setEnabled(enabled_flag)

        if status_tip is not None:
            action.setStatusTip(status_tip)

        if whats_this is not None:
            action.setWhatsThis(whats_this)

        if add_to_toolbar:
            self.toolbar.addAction(action)

        if add_to_menu:
            self.iface.addPluginToMenu(
                self.menu,
                action)

        self.actions.append(action)

        return action

    def initGui(self):
        """Create the menu entries and toolbar icons inside the QGIS GUI."""

        icon_path = ':/plugins/SEMImagePlacer/icon.png'
        self.add_action(
            icon_path,
            text=self.tr(u'Place SEM Images'),
            callback=self.run,
            parent=self.iface.mainWindow())

    def unload(self):
        """Remove the plugin menu item and icon from QGIS GUI."""
        for action in self.actions:
            self.iface.removePluginMenu(
                self.tr(u'&SEM Image Placer'),
                action)
            self.iface.removeToolBarIcon(action)

    def run(self):
        if self.selectingRefFlag is not 0:
            self.setUpSelectiongRef(self.selectingRefFlag)
            return True

        self.init_vars()
        self.projBehav = QSettings().value(u'Projections/defaultBehaviour')
        self.imgPaths = QFileDialog.getOpenFileNames(self.iface.mainWindow(),
                                                     'Select image files',
                                                     '~/',
                                                     'Images (*.tif *.jpg *.png)')
        if not self.imgPaths:
            return False

        if self.projBehav == u'prompt':
            proj_selector = QgsGenericProjectionSelector()
            proj_selector.exec_()
            self.crsId, self.crsType = self.getCrsInfo(
                proj_selector.selectedAuthId())
        else:
            self.crsId = self.canvas.mapRenderer().destinationCrs().srsid()
            self.crsType = self.canvas.mapRenderer().destinationCrs().CrsType()

        self.disableProjectionDialog()
        self.imgData = map(self.parseSEMTextFile, self.imgPaths)
        my_list = self.simpleImageLoading(self.imgData, 0, 0)
        map(self.saveWorldFile, my_list)
        self.layers = map(self.addImageFiles, my_list)
        self.enableProjectionDialog()

        self.dlg.show()
        result = self.dlg.exec_()
        if result:
            pass
            # self.canvas.setCanvasColor(QColor('black'))

    """
    Functions
    """

    def connects(self):
        QObject.connect(self.dlg.Btn_1,
                        SIGNAL('clicked(bool)'), self.doSomething)
        QObject.connect(self.dlg.Btn_Ref1,
                        SIGNAL('clicked(bool)'), self.selectRef1)
        QObject.connect(self.dlg.Btn_Ref2,
                        SIGNAL('clicked(bool)'), self.selectRef2)
        QObject.connect(self.clickTool,
                        SIGNAL("canvasClicked(const QgsPoint &, Qt::MouseButton)"),
                        self.selectPoints)
        QObject.connect(self.dlg, SIGNAL("closeEvent()"), self.init_vars)

    def disconnects(self):
        QObject.disconnect(self.dlg.Btn_1,
                           SIGNAL('clicked(bool)'), self.doSomething)
        QObject.disconnect(self.dlg.Btn_Ref1,
                           SIGNAL('clicked(bool)'), self.selectRef1)
        QObject.disconnect(self.dlg.Btn_Ref2,
                           SIGNAL('clicked(bool)'), self.selectRef2)
        QObject.disconnect(self.clickTool,
                           SIGNAL(
                               "canvasClicked(const QgsPoint &, Qt::MouseButton)"),
                           self.selectPoints)
        QObject.disconnect(self.dlg, SIGNAL("closeEvent()"), self.init_vars)

    def init_vars(self):
        self.referencePoints = {'1': {'from': [None, None],
                                      'to': [None, None]},
                                '2': {'from': [None, None],
                                      'to': [None, None]}}
        self.selectingRefFlag = 0
        self.layers = []
        self.imgPaths = []
        self.imgData = []

    def drawStatusBar(self, text):
        self.iface.mainWindow().statusBar().showMessage(str(text))

    def getCrsInfo(self, authId):
        crs_type, crsId = authId.split(':')
        if crs_type is 'EPSG':
            crs_type = QgsCoordinateReferenceSystem.EpsgCrsId
        elif crs_type is 'USER':
            crs_type = QgsCoordinateReferenceSystem.InternalCrsId
        else:
            crs_type = QgsCoordinateReferenceSystem.PostgisCrsId
        return [int(crsId), crs_type]

    def selectPoints(self, point, button):
        ref = self.referencePoints[str(self.selectingRefFlag)]
        if ref['from'][0] is None:
            ref['from'][0] = point.x()
            ref['from'][1] = point.y()
            if ref['to'][0] is None:
                stat_bar_txt = u'Selecting reference point {} "To"'
                self.drawStatusBar(stat_bar_txt.format(self.selectingRefFlag))
        else:
            ref['to'][0] = point.x()
            ref['to'][1] = point.y()
            self.finishSelectingRef()

    def selectRef1(self):
        self.selectingRefFlag = 1
        self.referencePoints['1']['from'] = [None, None]
        self.referencePoints['1']['to'] = [None, None]
        self.setUpSelectiongRef(self.selectingRefFlag)

    def selectRef2(self):
        self.selectingRefFlag = 2
        self.referencePoints['2']['from'] = [None, None]
        self.referencePoints['2']['to'] = [None, None]
        self.setUpSelectiongRef(self.selectingRefFlag)

    def setUpSelectiongRef(self, refNumber):
        if self.referencePoints[str(refNumber)]['from'][0] is None:
            point_name = 'From'
        else:
            point_name = 'To'
        stat_bar_txt = u'Selecting reference point {} "{}"'
        self.drawStatusBar(stat_bar_txt.format(refNumber, point_name))
        self.dlg.setWindowState(Qt.WindowMinimized)
        self.selectingRefFlag = refNumber
        self.canvas.setMapTool(self.clickTool)

    def finishSelectingRef(self):
        # refNumber = str(self.selectingRefFlag)
        self.canvas.unsetMapTool(self.clickTool)
        if self.selectingRefFlag == 1:
            self.dlg.Txt_Ref1From.setText(', '.join(map(str,
                                                        self.referencePoints['1']['from'])))
            self.dlg.Txt_Ref1To.setText(', '.join(map(str,
                                                      self.referencePoints['1']['to'])))
        else:
            self.dlg.Txt_Ref2From.setText(', '.join(map(str,
                                                        self.referencePoints['2']['from'])))
            self.dlg.Txt_Ref2To.setText(', '.join(map(str,
                                                      self.referencePoints['2']['to'])))
        self.dlg.setWindowState(Qt.WindowActive)
        self.calcRotationOffset()
        self.drawStatusBar('')

    def calcRotationOffset(self):
        if self.referencePoints['1']['from'][0] is not None and \
           self.referencePoints['2']['from'][0] is not None:
            center = self.getCenter(self.imgData)

            ref1_to = self.referencePoints['1']['to']
            ref1_from = self.referencePoints['1']['from']
            ref2_to = self.referencePoints['2']['to']
            ref2_from = self.referencePoints['2']['from']

            # rotation degree
            deg1 = atan2(ref1_from[0] - ref2_from[0],
                         ref1_from[1] - ref2_from[1])
            deg2 = atan2(ref1_to[0] - ref2_to[0], ref1_to[1] - ref2_to[1])
            rad_rotate = deg1 - deg2
            rotation_degree = degrees(rad_rotate)

            ref1_from_rotated = self.rotateCoordinates(ref1_from[0] - center['x'],
                                                       ref1_from[1] -
                                                       center['y'],
                                                       sin(rad_rotate),
                                                       cos(rad_rotate))
            offset_x = ref1_to[0] - (ref1_from_rotated[0] + center['x'])
            offset_y = ref1_to[1] - (ref1_from_rotated[1] + center['y'])

            self.dlg.Txt_OffsetX.setText(str(offset_x))
            self.dlg.Txt_OffsetY.setText(str(offset_y))
            self.dlg.SpnBox_Rotation.setValue(rotation_degree)

    def deleteLayers(self):
        map(lambda x: QgsMapLayerRegistry.instance().removeMapLayer(x),
            self.layers)
        self.layers = []

    def doSomething(self):
        self.disableProjectionDialog()
        self.deleteLayers()
        rotation_deg = float(self.dlg.SpnBox_Rotation.value())
        offset_x = float(self.dlg.Txt_OffsetX.text())
        offset_y = float(self.dlg.Txt_OffsetY.text())
        adj_data = self.adjustCoordinates(self.imgData,
                                          rotation_deg,
                                          offset_x,
                                          offset_y)
        map(self.saveWorldFile, adj_data)
        map(self.makeClippedImages, adj_data)
        map(self.addImageFiles, adj_data)
        # center = self.getCenter(adj_data)

        self.dlg.prgBar_1.setValue(100)
        self.init_vars()
        self.dlg.close()
        self.enableProjectionDialog()

    def makeClippingShapeFiles(self, inputData):
        r = 0.9333333  # image height w/o scale bar (information area)
        _sin = sin(inputData['radian'])
        _cos = cos(inputData['radian'])

        points = []
        points.append([0, 0])
        points.append([0, -1 * inputData['h'] * r])
        points.append([inputData['w'], -1 * inputData['h'] * r])
        points.append([inputData['w'], 0])
        points.append([0, 0])
        for k, point in enumerate(points):
            points[k] = self.rotateCoordinates(point[0], point[1], _sin, _cos)
        for k, point in enumerate(points):
            points[k] = QgsPoint(point[0] + inputData['x'],
                                 point[1] + inputData['y'])

        clipping_node = QgsVectorLayer('polygon', inputData['name'], 'memory')
        clipping_node.startEditing()
        data_provider = clipping_node.dataProvider()
        # add a feature
        feture = QgsFeature()
        feture.setGeometry(QgsGeometry.fromPolygon([points]))
        data_provider.addFeatures([feture])
        clipping_node.commitChanges()
        clipping_node.updateExtents()

        """ save shape file """
        clip_dir = os.path.join(inputData['dir'], 'clip')
        if not os.path.exists(clip_dir):
            os.makedirs(clip_dir)
        shape_file_path = os.path.join(clip_dir, inputData['name'] + '.shp')
        error = QgsVectorFileWriter.writeAsVectorFormat(clipping_node,
                                                        shape_file_path,
                                                        'CP1250',
                                                        None,
                                                        'ESRI Shapefile')

    def getCenter(self, inputData):
        x = []
        y = []
        for img in inputData:
            w = img['w'] * img['ps'] / 2
            h = img['h'] * img['ps'] / 2
            x.append(img['x'] - w)
            x.append(img['x'] + w)
            y.append(img['y'] - h)
            y.append(img['y'] + h)

        return {
            'x': min(x) + (max(x) - min(x)) / 2,
            'y': min(y) + (max(y) - min(y)) / 2,
        }

    def getMaxMin(self, listOfDictionary, index):
        maxmin = map(lambda x: x[index], listOfDictionary)
        max_value = max(maxmin)
        min_value = min(maxmin)
        return {'max': max_value,
                'min': min_value,
                'center': min_value + (max_value - min_value) / 2.0}

    def simpleImageLoading(self, inData, offsetX=0, offsetY=0):
        out = []
        for img in inData:
            tmp = dict(img)
            tmp['original_x'] = tmp['x']
            tmp['original_y'] = tmp['y']
            tmp['original_ps'] = tmp['ps']
            tmp['original_w'] = tmp['w']
            tmp['original_h'] = tmp['h']
            tmp['rotate_x'] = 0.0
            tmp['rotate_y'] = 0.0
            tmp['h'] *= tmp['ps']
            tmp['w'] *= tmp['ps']
            tmp['x'] -= tmp['w'] / 2.0 + offsetX
            tmp['y'] += tmp['h'] / 2.0 + offsetY
            out.append(tmp)
        return out

    def adjustCoordinates(self, imgData,
                          rotation_degree=0, offsetX=0, offsetY=0):
        deg = radians(rotation_degree)
        _sin = sin(deg)
        _cos = cos(deg)
        out = []

        center = self.getCenter(imgData)

        for img in imgData:
            tmp = dict(img)
            tmp['radian'] = deg
            tmp['original_x'] = tmp['x']
            tmp['original_y'] = tmp['y']
            tmp['original_ps'] = tmp['ps']
            tmp['original_w'] = tmp['w']
            tmp['original_h'] = tmp['h']
            tmp['rotate_x'] = tmp['ps'] * _sin
            tmp['rotate_y'] = tmp['ps'] * _sin
            tmp['h'] = tmp['h'] * tmp['ps']
            tmp['w'] = tmp['w'] * tmp['ps']
            tmp['x'] = tmp['x'] - tmp['w'] / 2.0
            tmp['y'] = tmp['y'] + tmp['h'] / 2.0
            rc = self.rotateCoordinates(tmp['x'] - center['x'],
                                        tmp['y'] - center['y'], _sin, _cos)
            tmp['x'] = rc[0] + offsetX + center['x']
            tmp['y'] = rc[1] + offsetY + center['y']
            tmp['ps'] = _cos * tmp['ps']
            tmp['clip'] = os.path.join(tmp['dir'],
                                       'clip',
                                       tmp['name'] + '_clip.tif')
            out.append(tmp)
        return out

    def makeClippedImages(self, img):
        shape_file_path = os.path.join(
            img['dir'], 'clip', img['name'] + '.shp')
        img_path = os.path.join(img['dir'], img['img'])
        destnodata = '-dstnodata 0'  # black pixcels become transparent
        destnodata = ''  # just cropped out
        tmp = r'"{0}" -overwrite -s_srs "{1}" -t_srs "{1}" {2} -q ' \
            + r'-cutline "{3}" -dstalpha -of GTiff "{4}" "{5}"'
        self.checkandDeleteFiles(img['clip'])
        self.makeClippingShapeFiles(img)
        cmd = tmp.format(self.gdalwarp, self.crs, destnodata,
                         shape_file_path, img_path, img['clip'])

        return subprocess.call(cmd, shell=True)

    def rotateCoordinates(self, x, y, _sin, _cos):
        rx = x * _cos - y * _sin
        ry = x * _sin + y * _cos
        return [rx, ry]

    def saveWorldFile(self, imgData):
        out = []
        try:
            rotatex = imgData['rotate_x']
            rotatey = imgData['rotate_y']
        except:
            rotatex = 0.0
            rotatey = 0.0
        out.append(str(imgData['ps']))         # pixel size x
        out.append(str(rotatex))               # rotation x
        out.append(str(rotatey))               # rotation y
        out.append(str(imgData['ps'] * -1.0))  # pixel size y
        out.append(str(imgData['x'] * 1.0))    # x coordinates
        out.append(str(imgData['y']))          # y coordinates
        worldfile_path = os.path.join(imgData['dir'],
                                      imgData['name'] + '.' + imgData['world'])
        f = open(worldfile_path, 'w')
        f.write('\n'.join(out))
        f.close()

        return worldfile_path

    def addPoint(self, point, nameLayer):
        center_point = QgsVectorLayer('Point', nameLayer, 'memory')
        center_point.startEditing()
        data_provider = center_point.data_provider()
        feture = QgsFeature()
        feture.setGeometry(QgsGeometry.fromPoint(QgsPoint(point[0], point[1])))
        data_provider.addFeatures([feture])
        center_point.commitChanges()
        center_point.updateExtents()

        QgsMapLayerRegistry.instance().addMapLayers([center_point])

    def disableProjectionDialog(self):
        self.projBehav = QSettings().value(u'Projections/defaultBehaviour')
        QSettings().setValue(u'Projections/defaultBehaviour', u'useGlobal')

    def enableProjectionDialog(self):
        QSettings().setValue(u'Projections/defaultBehaviour', self.projBehav)

    def addImgLayer(self, imgPath, imgName):
        rlayer = QgsRasterLayer(imgPath, imgName)
        rlayer.setCrs(QgsCoordinateReferenceSystem(self.crsId, self.crsType))
        QgsMapLayerRegistry.instance().addMapLayer(rlayer)

        return rlayer

    def addImageFiles(self, imgData):
        clip_flag = True
        try:
            img_path = imgData['clip']
        except:
            clip_flag = False
            img_path = os.path.join(imgData['dir'], imgData['img'])

        l = self.addImgLayer(img_path, imgData['name'])

        if clip_flag:
            l.setDrawingStyle('PalettedColor')
            # clip out scale bars
            lr = l.renderer()
            lr.setAlphaBand(2)

        return l.id()

    def checkandDeleteFiles(self, path):
        file_name = os.path.basename(path).split(os.extsep)[0].replace('_clip',
                                                                       '')
        file_dir = os.path.dirname(path)
        exts = ['shp', 'cpg', 'dbf', 'prj', 'shx', 'qpj']
        if os.path.exists(path):
            try:
                os.remove(path)
                for ext in exts:
                    try:
                        os.remove(os.path.join(
                            file_dir, file_name + '.' + ext))
                    except:
                        pass
            except:
                pass

    def sanitizePath(self, path):
        path = os.path.expandvars(os.path.expanduser(path))
        return os.path.abspath(path)

    def getWorldFileExt(self, ext):
        return str(ext[0]) + str(ext[2]) + 'w' if len(ext) == 3 else ext + 'w'

    def parseSEMTextFile(self, imgFilePath):
        img_path = self.sanitizePath(imgFilePath)
        tmp = img_path.split(os.extsep)
        img_name = os.path.basename(tmp[0])
        img_dir = os.path.dirname(img_path)
        img_txt = img_name + '.txt'
        f = open(os.path.join(img_dir, img_txt), 'r')
        m = re.compile(
            r'ImageName=(?P<name>.*\.[a-zA-Z0-9]+).*'
            r'DataSize=(?P<width>\d+)x(?P<height>\d+).*'
            r'PixelSize=(?P<pxsize>[\d\.]+).*'
            r'StagePositionX=(?P<x>\d+).*'
            r'StagePositionY=(?P<y>\d+).*', re.M | re.S)
        p = m.search(f.read())
        if not p:
            return None

        # units are micron
        return {'img': p.group('name'),
                'world': self.getWorldFileExt(tmp[1]),
                'dir': str(img_dir),
                'name': str(img_name),
                'x': float(p.group('x')) / -1000.0,
                'y': float(p.group('y')) / 1000.0,
                'ps': float(p.group('pxsize')) / 1000.0,
                'w': int(p.group('width')),
                'h': int(p.group('height'))}
