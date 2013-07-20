var engine = function(io) {
	var t = new iio.Text('browsercraft', io.canvas.center);
	t.setFont('30px Consolas');
	t.setTextAlign('center');
	t.setFillStyle('black');
	io.addObj(t);
};

module.exports = {
	start: function() {
		iio.start(engine);
	}
};

