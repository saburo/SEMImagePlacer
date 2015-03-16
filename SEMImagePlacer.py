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
from PyQt4.QtCore import QSettings, QTranslator, qVersion, QCoreApplication, QObject, SIGNAL, Qt
from PyQt4.QtGui import QAction, QIcon, QFileDialog, QColor, QMessageBox
from qgis.core import QgsGeometry, QgsPoint, QgsRasterLayer, QgsRasterTransparency, QgsVectorLayer, QgsFeature, QgsVectorFileWriter, QgsMapLayerRegistry, QgsCoordinateReferenceSystem
from qgis.gui import QgsMapToolEmitPoint, QgsGenericProjectionSelector
# Initialize Qt resources from file resources.py
import resources_rc
# Import the code for the dialog
from SEMImagePlacer_dialog import SEMImagePlacerDialog

import os.path, os
import sys, re, time, subprocess
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
        self.gdalwarp = os.path.join(QSettings().value('GdalTools/gdalPath'), 'gdalwarp')
        self.crs = "+proj=longlat +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs"
        self.selectingRefFlag = 0
        self.imgData = []
        self.layers = []
        self.referencePoints = {'1':{'from': [None,None], 'to': [None,None]}, '2': {'from': [None,None], 'to':[None,None]}}
        self.connects()

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
        """Removes the plugin menu item and icon from QGIS GUI."""
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
        self.imgPaths = QFileDialog.getOpenFileNames(self.iface.mainWindow(), 'Select image files', '~/Desktop/', 'Images (*.tif *.jpg *.png)')
        if not self.imgPaths: return False

        self.imgData = map(self.parseSEMTextFile, self.imgPaths)
        # center = self.getCenter(self.imgData)
        myList = self.simpleImageLoading(self.imgData, 0, 0)
        map(self.saveWorldFile, myList)
        self.layers = map(self.addImageFiles, myList)
        center = self.getCenter(self.imgData)
        # self.addPoint([center['x'], center['y']], 'pre-center')

        # show the dialog
        self.dlg.show()
        # Run the dialog event loop
        result = self.dlg.exec_()
        # See if OK was pressed
        if result:
            pass
            # self.canvas.setCanvasColor(QColor('black'))


#####################################################################
## Functions
#####################################################################

    def connects(self):
        QObject.connect(self.dlg.Btn_1, SIGNAL('clicked(bool)'), self.doSomething)
        QObject.connect(self.dlg.Btn_Ref1, SIGNAL('clicked(bool)'), self.selectRef1)
        QObject.connect(self.dlg.Btn_Ref2, SIGNAL('clicked(bool)'), self.selectRef2)
        QObject.connect(self.clickTool, SIGNAL("canvasClicked(const QgsPoint &, Qt::MouseButton)"), self.selectPoints)
        QObject.connect(self.dlg, SIGNAL("closeEvent()"), self.init_vars)

    def disconnects(self):
        QObject.disconnect(self.dlg.Btn_1, SIGNAL('clicked(bool)'), self.doSomething)
        QObject.disconnect(self.dlg.Btn_Ref1, SIGNAL('clicked(bool)'), self.selectRef1)
        QObject.disconnect(self.dlg.Btn_Ref2, SIGNAL('clicked(bool)'), self.selectRef2)
        QObject.disconnect(self.clickTool, SIGNAL("canvasClicked(const QgsPoint &, Qt::MouseButton)"), self.selectPoints)
        QObject.disconnect(self.dlg, SIGNAL("closeEvent()"), self.init_vars)

    def init_vars(self):
        self.referencePoints = {'1':{'from': [None,None], 'to': [None,None]}, '2': {'from': [None,None], 'to':[None,None]}}
        self.selectingRefFlag = 0
        self.layers = []
        self.imgPaths = []
        self.imgData = []

    def selectPoints(self, point, button):
        ref = self.referencePoints[str(self.selectingRefFlag)]
        if ref['from'][0] is None:
            ref['from'][0] = point.x()
            ref['from'][1] = point.y()
        else:
            ref['to'][0] = point.x()
            ref['to'][1] = point.y()
            self.finishSelectingRef()
        QMessageBox.information(self.iface.mainWindow(), 'test', '{}, {}'.format(point.x(), point.y()))

    def selectRef1(self):
        self.selectingRefFlag = 1
        self.setUpSelectiongRef(self.selectingRefFlag)

    def selectRef2(self):
        self.selectingRefFlag = 2
        self.setUpSelectiongRef(self.selectingRefFlag)

    def setUpSelectiongRef(self, refNumber):
        res = QMessageBox.information(self.iface.mainWindow(),"Select Points", 'Click the points', 'OK', 'Cancel')
        if res: return False
        self.dlg.setWindowState(Qt.WindowMinimized)
        self.selectingRefFlag = refNumber
        self.referencePoints[str(refNumber)]['from'] = [None, None]
        self.referencePoints[str(refNumber)]['to'] = [None, None]
        self.canvas.setMapTool(self.clickTool)

    def finishSelectingRef(self):
        refNumber = str(self.selectingRefFlag)
        self.canvas.unsetMapTool(self.clickTool)
        if self.selectingRefFlag == 1:
            self.dlg.Txt_Ref1From.setText(', '.join(map(str, self.referencePoints['1']['from'])))
            self.dlg.Txt_Ref1To.setText(', '.join(map(str, self.referencePoints['1']['to'])))
        else:
            self.dlg.Txt_Ref2From.setText(', '.join(map(str, self.referencePoints['2']['from'])))
            self.dlg.Txt_Ref2To.setText(', '.join(map(str, self.referencePoints['2']['to'])))
        self.dlg.setWindowState(Qt.WindowActive)
        self.calcRotationOffset()

    def calcRotationOffset(self):
        if self.referencePoints['1']['from'][0] is not None and self.referencePoints['2']['from'][0] is not None:
            center = self.getCenter(self.imgData)

            ref1To   = self.referencePoints['1']['to']
            ref1From = self.referencePoints['1']['from']
            ref2To   = self.referencePoints['2']['to']
            ref2From = self.referencePoints['2']['from']

            # rotation degree
            radRotate = atan2(ref1From[0] - ref2From[0], ref1From[1] - ref2From[1]) - atan2(ref1To[0] - ref2To[0], ref1To[1] - ref2To[1])
            rotationDegree = degrees(radRotate)

            ref1From_rotated = self.rotateCoordinates(ref1From[0] - center['x'],
                                                        ref1From[1] - center['y'],
                                                        sin(radRotate),
                                                        cos(radRotate))
            offsetX = ref1To[0] - (ref1From_rotated[0] + center['x'])
            offsetY = ref1To[1] - (ref1From_rotated[1] + center['y'])

            self.dlg.Txt_OffsetX.setText(str(offsetX))
            self.dlg.Txt_OffsetY.setText(str(offsetY))
            self.dlg.SpnBox_Rotation.setValue(rotationDegree)

    def deleteLayers(self):
        map(lambda x: QgsMapLayerRegistry.instance().removeMapLayer(x), self.layers)
        self.layers = []

    def doSomething(self):
        self.deleteLayers()
        rotation_deg = float(self.dlg.SpnBox_Rotation.value())
        offsetX = float(self.dlg.Txt_OffsetX.text())
        offsetY = float(self.dlg.Txt_OffsetY.text())
        adjData = self.adjustCoordinates(self.imgData, rotation_deg, offsetX, offsetY)
        map(self.saveWorldFile, adjData)
        map(self.makeClippedImages, adjData)
        layers = map(self.addImageFiles, adjData)
        center = self.getCenter(adjData)
        # self.addPoint([center['x'], center['y']], 'post-center')

        self.dlg.prgBar_1.setValue(100)
        self.init_vars()
        self.dlg.close()

    def makeClippingShapeFiles(self, inputData):
        r = 0.9333333 # scale var offset
        sinD = sin(inputData['radian'])
        cosD = cos(inputData['radian'])

        points = []
        points.append([0, 0])
        points.append([0, -1 * inputData['h'] * r])
        points.append([inputData['w'], -1 * inputData['h'] * r])
        points.append([inputData['w'], 0])
        points.append([0, 0])
        for k, point in enumerate(points):
            points[k] = self.rotateCoordinates(point[0], point[1], sinD, cosD)
        for k, point in enumerate(points):
            points[k] = QgsPoint(point[0] + inputData['x'], point[1] + inputData['y'])

        clippingNode = QgsVectorLayer('polygon', inputData['name'], 'memory')
        clippingNode.startEditing()
        dataProvider = clippingNode.dataProvider()
        # add a feature
        feture = QgsFeature()
        feture.setGeometry(QgsGeometry.fromPolygon([points]))
        dataProvider.addFeatures([feture])
        clippingNode.commitChanges()
        clippingNode.updateExtents()

        # save shape file
        if not os.path.exists(os.path.join(inputData['dir'], 'clip')):
            os.makedirs(os.path.join(inputData['dir'], 'clip'))
        error = QgsVectorFileWriter.writeAsVectorFormat(clippingNode,
                                                        os.path.join(inputData['dir'], 'clip', inputData['name'] + '.shp'),
                                                        'CP1250',
                                                        None,
                                                        'ESRI Shapefile')
        # print 'making a clipping vector layer: ' + str(error)

    def getCenter(self, inputData):
        tmpX = []
        tmpY = []
        for img in inputData:
            w = img['w'] * img['ps'] / 2
            h = img['h'] * img['ps'] / 2
            tmpX.append(img['x'] - w)
            tmpX.append(img['x'] + w)
            tmpY.append(img['y'] - h)
            tmpY.append(img['y'] + h)

        return {
            'x': min(tmpX) + (max(tmpX) - min(tmpX)) / 2,
            'y': min(tmpY) + (max(tmpY) - min(tmpY)) / 2,
        }

    def getMaxMin(self, listOfDictionary, index):
        maxmin = map(lambda x: x[index], listOfDictionary)
        maxValue = max(maxmin)
        minValue = min(maxmin)
        return {'max': maxValue, 'min': minValue, 'center': minValue + (maxValue - minValue) / 2.0}

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

    def adjustCoordinates(self, imgData, rotation_degree=0, offsetX=0, offsetY=0):
        deg = radians(rotation_degree)
        sinD = sin(deg)
        cosD = cos(deg)
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
            tmp['rotate_x'] = tmp['ps'] * sinD
            tmp['rotate_y'] = tmp['ps'] * sinD
            tmp['h'] = tmp['h'] * tmp['ps']
            tmp['w'] = tmp['w'] * tmp['ps']
            # tmp['h'] = cosD * tmp['h'] + sinD * tmp['w'] # new
            # tmp['w'] = cosD * tmp['w'] + sinD * tmp['h'] # new
            tmp['x'] = tmp['x'] - tmp['w'] / 2.0
            tmp['y'] = tmp['y'] + tmp['h'] / 2.0
            rc = self.rotateCoordinates(tmp['x'] - center['x'], tmp['y'] - center['y'], sinD, cosD)
            tmp['x'] = rc[0] + offsetX + center['x']
            tmp['y'] = rc[1] + offsetY + center['y']
            tmp['ps'] = cosD * tmp['ps']
            tmp['clip'] = os.path.join(tmp['dir'], 'clip', tmp['name'] + '_clip.tif')
            out.append(tmp)
        return out

    def makeClippedImages(self, img):
        shapefilepath = os.path.join(img['dir'], 'clip', img['name'] + '.shp')
        imgPath = os.path.join(img['dir'], img['img'])
        destnodata = '-dstnodata 0' # delete black -> transparent
        destnodata = '' # just cropped out
        form = r'"{0}" -overwrite -s_srs "{1}" -t_srs "{1}" {2} -q -cutline "{3}" -dstalpha -of GTiff "{4}" "{5}"'
        self.checkandDeleteFiles(img['clip'])
        self.makeClippingShapeFiles(img)
        cmd = form.format(self.gdalwarp, self.crs, destnodata, shapefilepath, imgPath, img['clip'])
        # # destnodata = ''
        res = subprocess.call(cmd, shell=True)

    def rotateCoordinates(self, x, y, sinD, cosD):
        rx = x * cosD - y * sinD
        ry = x * sinD + y * cosD
        return [rx, ry]

    def saveWorldFile(self, imgData):
        out = []
        try:
            rotatex = imgData['rotate_x']
            rotatey = imgData['rotate_y']
        except:
            rotatex = 0.0
            rotatey = 0.0
        out.append(str(imgData['ps'])) # pixel size x
        out.append(str(rotatex)) # rotation x
        out.append(str(rotatey)) # rotation y
        out.append(str(imgData['ps'] * -1.0)) # pixel size y
        out.append(str(imgData['x'] * 1.0)) # x coordinates
        out.append(str(imgData['y'])) # y coordinates
        worldfilePath = os.path.join(imgData['dir'], imgData['name'] + '.' + imgData['world'])
        f = open(worldfilePath, 'w')
        f.write('\n'.join(out))
        f.close()

        return worldfilePath

    def addPoint(self, point, nameLayer):
        centerPoint = QgsVectorLayer('Point', nameLayer, 'memory')
        centerPoint.startEditing()
        dataProvider = centerPoint.dataProvider()
        feture = QgsFeature()
        feture.setGeometry(QgsGeometry.fromPoint(QgsPoint(point[0], point[1])))
        dataProvider.addFeatures([feture])
        centerPoint.commitChanges()
        centerPoint.updateExtents()

        QgsMapLayerRegistry.instance().addMapLayers([centerPoint])

    def addImageFiles(self, imgData):
        clipFlag = True
        try:
            imgPath = imgData['clip']
        except:
            clipFlag = False
            imgPath = os.path.join(imgData['dir'], imgData['img'])
        self.iface.addRasterLayer(imgPath, imgData['name'])

        l = self.canvas.currentLayer()
        if clipFlag:
            l.setDrawingStyle('PalettedColor');
            # clip out scale bars
            lr = l.renderer()
            lr.setAlphaBand(2)
            #
            # lrt = lr.rasterTransparency()
            # x = QgsRasterTransparency.TransparentSingleValuePixel()
            # x.max = 0
            # x.min = 0
            # x.percentTransparent = 100
            # lrt.setTransparentSingleValuePixelList([x])
        return l.id()

    def checkandDeleteFiles(self, path):
        fileName = os.path.basename(path).split(os.extsep)[0].replace('_clip', '')
        fileDir = os.path.dirname(path)
        exts = ['shp', 'cpg', 'dbf', 'prj', 'shx', 'qpj']
        if os.path.exists(path):
            try:
                os.remove(path)
                for ext in exts:
                    try:
                        os.remove(os.path.join(fileDir, fileName + '.' + ext))
                    except:
                        pass
            except:
                pass

    def sanitizePath(self, path):
        path = os.path.expanduser(path)
        path = os.path.expandvars(path)
        return os.path.abspath(path)

    def getWorldFileExt(self, ext):
        if len(ext) == 3:
            return str(ext[0]) + str(ext[2]) + 'w'
        else:
            return ext + 'w'

    def parseSEMTextFile(self, imgFilePath):
        imgPath = self.sanitizePath(imgFilePath)
        tmp = imgPath.split(os.extsep)
        imgName = os.path.basename(tmp[0])
        imgDir = os.path.dirname(imgPath)
        imgText = imgName + '.txt'
        f = open(os.path.join(imgDir, imgText), 'r')
        m = re.compile(
            r'ImageName=(?P<name>.*\.[a-zA-Z0-9]+).*'
            r'DataSize=(?P<width>\d+)x(?P<height>\d+).*'
            r'PixelSize=(?P<pxsize>[\d\.]+).*'
            r'StagePositionX=(?P<x>\d+).*'
            r'StagePositionY=(?P<y>\d+).*'
            , re.M|re.S)
        p = m.search(f.read())
        if not p:
            return None

        # units are micron
        return {'img': p.group('name'),
            'world': self.getWorldFileExt(tmp[1]),
            'dir': str(imgDir),
            'name': str(imgName),
            'x':  float(p.group('x')) / -1000.0,
            'y':  float(p.group('y')) / 1000.0,
            'ps': float(p.group('pxsize')) / 1000.0,
            'w': int(p.group('width')),
            'h': int(p.group('height'))}
