var wms_layers = [];

var lyr_NDVI_0 = new ol.layer.Image({
                            opacity: 1,
                            title: "NDVI",
                            
                            
                            source: new ol.source.ImageStatic({
                               url: "./layers/NDVI_0.png",
    attributions: ' ',
                                projection: 'EPSG:3857',
                                alwaysInRange: true,
                                imageExtent: [2172564.427813, 6425830.055277, 2266150.175951, 6501257.469868]
                            })
                        });

lyr_NDVI_0.setVisible(true);
var layersList = [lyr_NDVI_0];
