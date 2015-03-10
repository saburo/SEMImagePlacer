# -*- coding: utf-8 -*-
"""
/***************************************************************************
 SEMImagePlacer
                                 A QGIS plugin
 This plugin places SEM images by their own coordinates
                             -------------------
        begin                : 2015-03-03
        copyright            : (C) 2015 by Kouki Kitajima
        email                : saburo@geology.wisc.edu
        git sha              : $Format:%H$
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
 This script initializes the plugin, making it known to QGIS.
"""


# noinspection PyPep8Naming
def classFactory(iface):  # pylint: disable=invalid-name
    """Load SEMImagePlacer class from file SEMImagePlacer.

    :param iface: A QGIS interface instance.
    :type iface: QgsInterface
    """
    #
    from .SEMImagePlacer import SEMImagePlacer
    return SEMImagePlacer(iface)
