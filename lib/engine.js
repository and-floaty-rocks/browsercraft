var engine = function(io) {
	var t = new iio.Text('browsercraft', io.canvas.center);
	t.setFont('30px Consolas');
	t.setTextAlign('center');
	t.setFillStyle('black');
	io.addObj(t);

	io.setFramerate(60);

	io.activateDebugger();
};

module.exports = {
	start: function() {
		iio.start(engine);
	}
};

