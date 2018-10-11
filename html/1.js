google.charts.load('current', {
  packages: ['gauge']
}).then(function () {
  var data0 = google.visualization.arrayToDataTable([ ['Label', 'Value'],
                                                      ['Input', 0] ]);

  var data1 = google.visualization.arrayToDataTable([ ['Label', 'Value'],
                                                      ['Output', 0] ]);

  var data2 = google.visualization.arrayToDataTable([ ['Label', 'Value'], 
                                                      ['ADC', 0] ]);

        var options0 = {
          width: 400, height: 400,
          yellowFrom: 90, yellowTo: 100,
          greenFrom: 100, greenTo: 110,
          redFrom: 110, redTo: 125,
          minorTicks: 20,
          majorTicks: ['0', '20', '40', '60', '80', '100', '120', '140'],
          max: 140,
          animation:{duration: 500},
        };

        var options1 = {
          width: 400, height: 400,
          yellowFrom: 0, yellowTo: 3,
          greenFrom: 3, greenTo: 6,
          redFrom: 6, redTo: 15,
          minorTicks: 3,
          majorTicks: ['0', '3', '6', '9', '12', '15'],
          max: 15,
          animation:{duration: 500},
        };

        var options2 = {
          width: 400, height: 400,
          greenFrom: 167, greenTo: 172,
          min: 130,
          max: 190,
          minorTicks: 10,
          majorTicks: ['130','140', '150', '160', '170', '180', '190'],
          animation:{duration: 500},
        };

        var chart0 = new google.visualization.Gauge(document.getElementById('chart_in'));
        var chart1 = new google.visualization.Gauge(document.getElementById('chart_out'));
        var chart2 = new google.visualization.Gauge(document.getElementById('chart_adc'));

        // chart.draw(data, options);

        setInterval(function() {
          // Gets ADC value at every one second
          GetADC();
        }, 1000);

        function GetADC() {
          var xhttp = new XMLHttpRequest();
          xhttp.onreadystatechange = function() {
            if (this.readyState == 4 && this.status == 200) {
             var response = JSON.parse(xhttp.responseText);
             data0.setValue(0,1,response.Input);
             data1.setValue(0,1,response.Output);
             data2.setValue(0,1,response.ADC);
             chart0.draw(data0, options0);
             chart1.draw(data1, options1);
             chart2.draw(data2, options2);
            }
          };
          xhttp.open("GET", "/json", false);
          xhttp.send();
        }

})
