<?php
if(isset($_POST['nofsimulations'])){
 $numofsims = $_POST['nofsimulations'];
}
if(isset($_POST['nofnodes'])){
$numofnodes = $_POST['nofnodes'];
}
if(isset($_POST['nofuavs'])){
$numofuavs = $_POST['nofuavs'];
}
if(isset($_POST['thresholdl'])){
$threshold = $_POST['thresholdl'];
}
if(isset($_POST['stepsz_node'])){
$stepsz_node = $_POST['stepsz_node'];
}

if(isset($_POST['stepsize'])){
$stepsize =  $_POST['stepsize'];
}
if(isset($_POST['steps'])){
$steps =  $_POST['steps'];
}
if(isset($_POST['maxx'])){
$maxx =  $_POST['maxx'];
}
if(isset($_POST['maxy'])){
$maxy =  $_POST['maxy'];

}
if(isset($_POST['maxz'])){
$maxz =  $_POST['maxz'];
}
if(isset($_POST['localization_method'])){
	$localization_method =  $_POST['localization_method'];
} else {
	$localization_method = 0;
}

if(isset($_POST['filtering'])){
	$filtering =  $_POST['filtering'];
} else {	
	$filtering = 0;
};

if(isset($_POST['trajectory'])){
	$trajectory =  $_POST['trajectory'];
} else {	
	$trajectory = 0;
};
$data = array($numofsims, $numofnodes,$numofuavs,$threshold,$stepsize,$steps,$maxx,$maxy,$maxz, $stepsz_node, $localization_method, $filtering, $trajectory);
//echo json_decode(print_r(data));
//$data = array(1,100,1,-100,40,150,200,200,100,0, 0,0);
$result = shell_exec('python ./Simulation_updated.py '.json_encode(json_encode($data)));
$resultData = json_decode($result, true);
echo json_encode($resultData);
?>