
var hostname = "roboRIO-1736-FRC.local:5805" //Robot hostname
//var hostname = "localhost:5805" //Local Hostname

var dataSocket = new WebSocket("ws://"+hostname+"/rtplot")

var signal_names = []
var signal_units = []
var signal_display_names = []


var global_chart;


dataSocket.onopen = function (event) {
    document.getElementById("id01").innerHTML = "COM Status: Socket Opened.";
    document.getElementById("stop_btn").disabled = true;
    document.getElementById("start_btn").disabled = false;
};

dataSocket.onerror = function (error) {
    document.getElementById("id01").innerHTML = "COM Status: Error with socket. Reconnect to robot, open driver station, then refresh this page.";
    alert("ERROR from RT Plot: Robot Disconnected!!!\n\nAfter connecting to the robot, open the driver station, then refresh this page.");
    document.getElementById("stop_btn").disabled = true;
    document.getElementById("start_btn").disabled = true;
};

dataSocket.onclose = function (error) {
    document.getElementById("id01").innerHTML = "COM Status: Error with socket. Reconnect to robot, open driver station, then refresh this page.";
    alert("ERROR from RT Plot: Robot Disconnected!!!\n\nAfter connecting to the robot, open the driver station, then refresh this page.");
    document.getElementById("stop_btn").disabled = true;
    document.getElementById("start_btn").disabled = true;
};

dataSocket.onmessage = function (event) {
    var data = JSON.parse(event.data);
    if(data.type == "daq_update"){
        addDataToPlot(data.samples);
    } else if(data.type == "signal_list"){
        genSignalListTable(data.signals);
    }

};

function addDataToPlot(data){
    var sig_iter;
    var samp_iter;
    
    for(sig_iter = 0; sig_iter < data.length; sig_iter++){
        for(samp_iter = 0; samp_iter < data[sig_iter].samples.length; samp_iter++){
            global_chart.series[sig_iter].addPoint([parseFloat(data[sig_iter].samples[samp_iter].time), parseFloat(data[sig_iter].samples[samp_iter].val)],false,false,true);
        }
    }
    
    global_chart.redraw();
    
}

function genSignalListTable(arr){
    var i;
    var col_counter = 0;
    var SIGNALS_PER_ROW = 1; //meh. html is hard.
    signal_names = [];
    
    var out = "<table><tbody><tr>";
    
    for(i = 0; i < arr.length; i++){
        signal_names.push(arr[i].name);
        signal_units.push(arr[i].units);
        signal_display_names.push(arr[i].display_name);
        out += "<td><input type=\"checkbox\" name=\""+arr[i].name+"\" />"+arr[i].display_name+" (" + arr[i].units + ") </td>";
       
        if(col_counter >= (SIGNALS_PER_ROW-1)){
            //start a new row
            col_counter = 0;
            out += "</tr><tr>";
        } else {
            col_counter++;
        }
    }
    out +="</tr></tbody></table>";
    document.getElementById("id02").innerHTML = out;

}

function handleStartBtnClick(){
    var cmd = "start:";
    var temp_series = [];
    var units_to_yaxis_index = [];
    var yaxis_index = 0;
    
    //deep-copy the default chart options
	var options = $.extend(true, {}, dflt_options)
    
    //Destroy any existing chart.
	if(global_chart){
        //double check the user didn't click it by mistake.
        if(confirm('This will clear the current recording. Are you sure?')){
            global_chart.destroy();
        } else {
            return; //do nothing
        }
		
	}
	
    
    //Disable signal selection
    document.getElementById("clear_btn").disabled = true;
    document.getElementById("start_btn").disabled = true;
    for(i = 0; i < signal_names.length; i++){
        checkboxes = document.getElementsByName(signal_names[i]);
        for(var j=0, n=checkboxes.length;j<n;j++) {
            checkboxes[j].disabled = true;
        }
    }
    
    //Select only checked signals
    for(i = 0; i < signal_names.length; i++){
        checkboxes = document.getElementsByName(signal_names[i]);
        for(var j=0, n=checkboxes.length;j<n;j++) {
            
            //For all checked boxes...
            if(checkboxes[j].checked == true){
                //Assemble command for sending to server
                cmd += signal_names[i] + ",";
                
                //Handle grouping like-units signals on the same Y axis
                var unit = signal_units[i];
                if(!(unit in units_to_yaxis_index)){
                    units_to_yaxis_index[unit] = yaxis_index;
                    options.yAxis.push({title:{text:unit}, showEmpty:false});
                    yaxis_index++;
                }
                
                // set up chart for signals
                temp_series.push({name:signal_display_names[i],
                                  data:[],
                                  visible:true,
                                  visibility_counter:0,
                                  yAxis:units_to_yaxis_index[unit]
                                 });

            }
            

    
            
        }
    }
    
    
    //Create the Highcharts chart just before starting DAQ
        
    //Add all data to the chart
    $.each(temp_series, function(itemNo, element) {
        options.series.push(element);
    });
    //chart named after its sosurce file
    options.title.text = "Real-Time Data";
    //Create dat chart
    global_chart = new Highcharts.Chart(options);

    //Request data from robot
    dataSocket.send(cmd); 
    document.getElementById("stop_btn").disabled = false;
}

function handleStopBtnClick(){
    //Request stopping data from robot
    dataSocket.send("stop:"); 
    
    document.getElementById("stop_btn").disabled = true;
    
    //re-enable siagnal selection
    document.getElementById("clear_btn").disabled = false;
    document.getElementById("start_btn").disabled = false;
    for(i = 0; i < signal_names.length; i++){
        checkboxes = document.getElementsByName(signal_names[i]);
        for(var j=0, n=checkboxes.length;j<n;j++) {
            checkboxes[j].disabled = false;
        }
    }
}

function handleRefreshSignalsBtnClick(){
    dataSocket.send("get_list:"); 
}

function handleClearBtnClick(){
    var i;
    //Reset all checkboxes to unchecked.
    for(i = 0; i < signal_names.length; i++){
        checkboxes = document.getElementsByName(signal_names[i]);
        for(var j=0, n=checkboxes.length;j<n;j++) {
            checkboxes[j].checked = false;
        }
    }

}

/**************************************************************************************
 ** HIGHCHARTS SUPPORT
 **************************************************************************************/


var dflt_options =  {    

		credits: {
			enabled: false
		},
		chart: {
			zoomType: 'x',
			renderTo: 'container',
			animation: true,
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
			title: 'Time (sec)'
		},
		
		yAxis: [],
		
		legend: {
			layout: 'vertical',
            align: 'right',
            verticalAlign: 'top',
            borderWidth: 1,
            floating: true
			
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
					radius: 5
				},
				lineWidth: 3,
				threshold: null,
				animation: true,
			}
		},
		tooltip: {
			crosshairs: true,
			hideDelay: 0,
			shared: true,
			backgroundColor: null,
            snap: 30,
			borderWidth: 1,
			shadow: true,
			animation: true,
			useHTML: false,
			style: {
					padding: 0
				}
            },  

		series: []
	}