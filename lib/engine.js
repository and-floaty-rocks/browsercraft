var Interface, initGame;
var mouseRect;

Interface = {
	menu: function(io) {
		console.log('UI >> menu');
		var options = {};

		io.rmvFromGroup('ui', 1); //clear previous UI
		//io.draw(1);
		var spacing = 50;

		var t = new iio.Text('browsercraft', io.cnvs[1].width/2, spacing);
		t.setFont('30px Fresca');
		t.setFillStyle('black');
		t.setTextAlign('center');
		io.addToGroup('ui', t, 10, 1);

		options.play = new iio.Rect({x: 250, y: spacing*2}, 160, 45);
		options.play.text = new iio.Text('play', options.play.pos);
		options.play.callback = initGame;

		options.settings = new iio.Rect({x: 250, y: spacing*3}, 160, 45);
		options.settings.text = new iio.Text('settings', options.settings.pos);

		for(var ii in options) {
			var option = options[ii];

			option.setFillStyle('#5e3f6b');
			io.addToGroup('ui', option, 10, 1);

			option.text.setFont('30px Fresca')
				.setFillStyle('white')
				.setTextAlign('center')
				.translate(0,8);
			io.addToGroup('ui', option.text, 10, 1);
		}

		var clickHandler = function(event) {
			var pos = io.getEventPosition(event, 1);

			for(var ii in options) {
				var option = options[ii];

				if(option.contains(pos)) {
					console.log(option.text.text);
					if(option.callback) {
						io.cnvs[1].removeEventListener('mousedown', clickHandler);
						option.callback(io);
					}
				}
			}
		};
		io.cnvs[1].addEventListener('mousedown', clickHandler);
	},
	game: function (io) {
		console.log("UI >> game");

		//clear the group of all objects
		//io.rmvFromGroup('ui', 1); //needs iio core update to work
		var ui = io.getGroup('ui', 1); //workaround
		for(var ii=0, length=ui.length; ii<length; ii++) {
			ui.pop();
		}
		io.draw(1);

		var width = io.canvas.width;
		var height = io.canvas.height;

		var section = new iio.Rect(width/2, height*7/8, width, height/4);
		section.setFillStyle('#5e3f6b');
		io.addToGroup('ui', section, 10, 1);

		var map = new iio.Rect(width*1/8, height*7/8, width/4-30, height/4-30);
		map.setFillStyle('black');
		io.addToGroup('ui', map, 10, 1);

		var commands = new iio.Rect(width*7/8, height*7/8, width/4-30, height/4-30);
		commands.setFillStyle('black');
		io.addToGroup('ui', commands, 10, 1);
		commands.grid = new iio.Grid(width*3/4+15, height*3/4+15, 5, 3, commands.width/5, commands.height/3);
		commands.grid.setStrokeStyle('#5e3f6b');
		io.addToGroup('ui', commands.grid, 10, 1);

		var unit = new iio.Rect(width/2, height*7/8, width/2, height/4-30);
		unit.setFillStyle('black');
		io.addToGroup('ui', unit, 10, 1);

		var createResourceText = function(x, y, w, h, text) {
			var obj = new iio.Rect(x, y, w, h)
				.setFillStyle('#5e3f6b');
			obj.text = new iio.Text(text)
				.setFont('15px Consolas')
				.setTextAlign('center')
				.setFillStyle('white');

			obj.addObj(obj.text, false, io.ctxs[1], 27, 5);
			return io.addToGroup('ui', obj, 10, 1);
		};

		var minerals = createResourceText(width*7/8-40, 20, 20, 20, '500');
		var gas = createResourceText(width*7/8+30, 20, 20, 20, '500');
		var supply = createResourceText(width*7/8+100, 20, 20, 20, '5/13');
	}
};

var initGame = function initGame(io) {
	Interface.game(io);
	io.rmvAll(0);
	io.draw(0);

	var t = new iio.Text('pew pew', io.canvas.center);
	t.setFont('30px Fresca');
	t.setFillStyle('white');
	t.setTextAlign('center');
	io.addObj(t);
};

var ui = function ui(io) {
	io.addCanvas(1); // zIndex to put UI over game world
	io.addGroup('ui', 10, 1);

	mouseRect = io.addToGroup('mouse', new iio.Rect(0, 0, 10), 0, 1);

	io.cnvs[1].addEventListener('mousemove', function(event) {
		mouseRect.setPos(io.getEventPosition(event));
		//console.log(mouseRect);
	});

	//io.setFramerate(60, 1);

	Interface.menu(io);
};

var engine = function(io) {
	io.activateDebugger();

	io.setFramerate(60); //text doesn't redraw?

	io.setBGColor("green");
	var t = new iio.Text('that green game canvas', io.canvas.center);
	t.setFont('30px Fresca');
	t.setFillStyle('white');
	t.setTextAlign('center');
	io.addObj(t);

	ui(io);
};

module.exports = {
	start: function() {
		iio.start(engine);
	}
};

