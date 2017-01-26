//dataViewer.js
var global_chart;

//this set of presets is specific to 2015 - update as needed!
var presets = [["PDBMeasVoltage (V)", "PDBMeasCurrent (A)"],
               ["LaunchWheelCurrent (cmd)", "LaunchWheelActSpeed (RPM)", "LaunchWheelDesSpeed (RPM)"],
			   ["PneumaticPress (PSI)", "CompressorCurrent (A)"],
			   ["BattEstVoc (V)", "BattEstESR (ohm)"]];

var dflt_options =  {    

		credits: {
			enabled: false
		},
		chart: {
			zoomType: 'x',
			renderTo: 'container',
			animation: false,
			ignoreHiddenSeries: true,
			panning: true,
			panKey: 'shift',
			showAxes: true
		},
		title: {
			text: ''
		},
		xAxis: {
			type: 'linear',
			title: 'seconds (since boot)'
		},
		
		yAxis: [],
		
		legend: {
			enabled: true,
			itemMarginBottom: 2,
			itemMarginTop: 2,
			itemWidth: 300
			
		},
		
		exporting: {
			enabled: false
		},
		
		colors: ['#7cb5ec', '#43A348', '#90ed7d', '#f7a30c', '#8085e9', '#f15c80', '#e4d354', '#2b608f', '#a45b5b', '#91efe1',
		         '#6cb5ec', '#43C348', '#90ed7d', '#f6a34c', '#8085e9', '#315c90', '#e4d354', '#2b908f', '#345bfb', '#61e8e1',
				 '#5cb5ec', '#43F348', '#90ed7d', '#f7a38c', '#8085e9', '#f18c80', '#e4d354', '#2b208f', '#f45b5b', '#91e8e1',
				 '#4cb5ec', '#43B348', '#90ed7d', '#f7a3Ac', '#8085e9', '#f12c80', '#e4d054', '#2bF08f', '#a45bfb', '#41e851',
				 '#3cb5ec', '#430348', '#90ed7d', '#f7a3Fc', '#8099e9', '#f15c50', '#e4d354'
                ],
   
		plotOptions: {
			line: {
				marker: {
					radius: 2
				},
				lineWidth: 1,
				threshold: null,
				animation: false,
			}
		},
		tooltip: {
			crosshairs: true,
			hideDelay: 0,
			shared: true,
			backgroundColor: null,
			borderWidth: 0,
			shadow: true,
			animation: false,
			useHTML: false,
			style: {
					padding: 0
				}
            },

		series: []
	}


function handleFileSelect(files_in) {

    var fileobj = files_in[0];
    var temp_series = [];
    var units_to_yaxis_index = [];
    var yaxis_index = 0;


	//Destroy any existing chart.
	if(global_chart){
		global_chart.destroy();
	}
	
	//deep-copy the default chart options
	var options = $.extend(true, {}, dflt_options)


	var reader = new FileReader();
	reader.readAsText(fileobj);
    reader.onload = function(evt) {
    
		var all_lines = evt.target.result + '';
        var lines = all_lines.split('\n');
        var timestamp = 0;
        var plotter_index = 0;
		
		
        // Iterate over the lines and add categories or series
        $.each(lines, function(lineNo, line) {
            var items = line.split(',');
            
            // first line containes signal names. Ignore Time column.
            if (lineNo == 0) {
                plotter_index = 0;
                $.each(items, function(itemNo, item) {
                    if(itemNo > 0) {
                        if(item.length > 1){
                            temp_series.push({name:item.replace(/ /g,''),
                                              data:[],
                                              visible:false,
                                              visibility_counter:0,
                                              yAxis:0 //temp, will be updated once the actual unit is read.
                                             });
                            plotter_index++;
                        }
                    }
                });
            }
            
            // second line containes units. Ignore Time column.
            else if (lineNo == 1) {
                plotter_index = 0;
                $.each(items, function(itemNo, item) {
                    if(itemNo > 0){
                        if(item.length > 1){
                            var unit = item.replace(/ /g,'');
                            temp_series[plotter_index].name = temp_series[plotter_index].name + ' (' + unit + ')';
                            if(!(unit in units_to_yaxis_index)){
                                units_to_yaxis_index[unit] = yaxis_index;
                                options.yAxis.push({title:{text:unit}, showEmpty:false});
                                yaxis_index++;
                            }
                            temp_series[plotter_index].yAxis = units_to_yaxis_index[unit];
                            plotter_index++;
                        }
                    } 
                });
            }
            
            // the rest of the lines contain data with their name in the first 
            // position
            else {
                plotter_index = 0;
                $.each(items, function(itemNo, item) {
                    if (itemNo == 0) {
                        timestamp = parseFloat(item);
                    } else {
                        if(item.length > 1){
                            temp_series[plotter_index].data.push([timestamp, parseFloat(item)]);
                            plotter_index++;
                        }
                    }
                });
                
            }
            
            
            
        });
        
		//Add all data to the chart
        $.each(temp_series, function(itemNo, element) {
            options.series.push(element);
        });
		
		//chart named after its sosurce file
		options.title.text = fileobj.name;
		
		//Create dat chart
		global_chart = new Highcharts.Chart(options);
		

		
	};
};

function setPreset(preset_idx){
	if(global_chart){	
		hideAll();
		for(itemNo = 0; itemNo < global_chart.series.length; itemNo++){
			for(preset_iter_idx = 0; preset_iter_idx < presets[preset_idx].length; preset_iter_idx++){
				if(global_chart.series[itemNo].name == presets[preset_idx][preset_iter_idx]){
					global_chart.series[itemNo].setVisible(true,false);
				}
			}
		}
		global_chart.redraw();
	}
}


function hideAll(){
	if(global_chart){	
		for(itemNo = 0; itemNo < global_chart.series.length; itemNo++){
			global_chart.series[itemNo].setVisible(false,false);
		}
	}
	global_chart.redraw();
}
