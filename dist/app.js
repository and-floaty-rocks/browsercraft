;(function(e,t,n){function i(n,s){if(!t[n]){if(!e[n]){var o=typeof require=="function"&&require;if(!s&&o)return o(n,!0);if(r)return r(n,!0);throw new Error("Cannot find module '"+n+"'")}var u=t[n]={exports:{}};e[n][0].call(u.exports,function(t){var r=e[n][1][t];return i(r?r:t)},u,u.exports)}return t[n].exports}var r=typeof require=="function"&&require;for(var s=0;s<n.length;s++)i(n[s]);return i})({1:[function(require,module,exports){
(function(){/*global window:true*/
window.browsercraft = require('./browsercraft');
window.browsercraft.go_go();


})()
},{"./browsercraft":2}],2:[function(require,module,exports){
/*
 * browsercraft
 * andfloatyrocks.com
 *
 * Copyright (c) 2013 dstrek
 * Licensed under the MIT license.
 */

(function(exports) {
	'use strict';

	var engine = require('./engine');

  exports.awesome = function() {
    return 'awesome';
  };

	exports.go_go = function() {
		engine.start();
	};

}(typeof exports === 'object' && exports || this));


},{"./engine":3}],3:[function(require,module,exports){
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
		io.clearGroup('ui', 1); //needs iio core update to work
	/*	var ui = io.getGroup('ui', 1); //workaround
		for(var ii=0, length=ui.length; ii<length; ii++) {
			ui.pop();
		}
	*/	io.draw(1);

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


},{}]},{},[1])
//@ sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiZ2VuZXJhdGVkLmpzIiwic291cmNlcyI6WyIvVXNlcnMvZHN0cmVrL3d3dy9icm93c2VyY3JhZnQvbGliL2VudHJ5LmpzIiwiL1VzZXJzL2RzdHJlay93d3cvYnJvd3NlcmNyYWZ0L2xpYi9icm93c2VyY3JhZnQuanMiLCIvVXNlcnMvZHN0cmVrL3d3dy9icm93c2VyY3JhZnQvbGliL2VuZ2luZS5qcyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiO0FBQUE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ0xBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUN2QkE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQSIsInNvdXJjZXNDb250ZW50IjpbIihmdW5jdGlvbigpey8qZ2xvYmFsIHdpbmRvdzp0cnVlKi9cbndpbmRvdy5icm93c2VyY3JhZnQgPSByZXF1aXJlKCcuL2Jyb3dzZXJjcmFmdCcpO1xud2luZG93LmJyb3dzZXJjcmFmdC5nb19nbygpO1xuXG5cbn0pKCkiLCIvKlxuICogYnJvd3NlcmNyYWZ0XG4gKiBhbmRmbG9hdHlyb2Nrcy5jb21cbiAqXG4gKiBDb3B5cmlnaHQgKGMpIDIwMTMgZHN0cmVrXG4gKiBMaWNlbnNlZCB1bmRlciB0aGUgTUlUIGxpY2Vuc2UuXG4gKi9cblxuKGZ1bmN0aW9uKGV4cG9ydHMpIHtcblx0J3VzZSBzdHJpY3QnO1xuXG5cdHZhciBlbmdpbmUgPSByZXF1aXJlKCcuL2VuZ2luZScpO1xuXG4gIGV4cG9ydHMuYXdlc29tZSA9IGZ1bmN0aW9uKCkge1xuICAgIHJldHVybiAnYXdlc29tZSc7XG4gIH07XG5cblx0ZXhwb3J0cy5nb19nbyA9IGZ1bmN0aW9uKCkge1xuXHRcdGVuZ2luZS5zdGFydCgpO1xuXHR9O1xuXG59KHR5cGVvZiBleHBvcnRzID09PSAnb2JqZWN0JyAmJiBleHBvcnRzIHx8IHRoaXMpKTtcblxuIiwidmFyIEludGVyZmFjZSwgaW5pdEdhbWU7XG52YXIgbW91c2VSZWN0O1xuXG5JbnRlcmZhY2UgPSB7XG5cdG1lbnU6IGZ1bmN0aW9uKGlvKSB7XG5cdFx0Y29uc29sZS5sb2coJ1VJID4+IG1lbnUnKTtcblx0XHR2YXIgb3B0aW9ucyA9IHt9O1xuXG5cdFx0aW8ucm12RnJvbUdyb3VwKCd1aScsIDEpOyAvL2NsZWFyIHByZXZpb3VzIFVJXG5cdFx0Ly9pby5kcmF3KDEpO1xuXHRcdHZhciBzcGFjaW5nID0gNTA7XG5cblx0XHR2YXIgdCA9IG5ldyBpaW8uVGV4dCgnYnJvd3NlcmNyYWZ0JywgaW8uY252c1sxXS53aWR0aC8yLCBzcGFjaW5nKTtcblx0XHR0LnNldEZvbnQoJzMwcHggRnJlc2NhJyk7XG5cdFx0dC5zZXRGaWxsU3R5bGUoJ2JsYWNrJyk7XG5cdFx0dC5zZXRUZXh0QWxpZ24oJ2NlbnRlcicpO1xuXHRcdGlvLmFkZFRvR3JvdXAoJ3VpJywgdCwgMTAsIDEpO1xuXG5cdFx0b3B0aW9ucy5wbGF5ID0gbmV3IGlpby5SZWN0KHt4OiAyNTAsIHk6IHNwYWNpbmcqMn0sIDE2MCwgNDUpO1xuXHRcdG9wdGlvbnMucGxheS50ZXh0ID0gbmV3IGlpby5UZXh0KCdwbGF5Jywgb3B0aW9ucy5wbGF5LnBvcyk7XG5cdFx0b3B0aW9ucy5wbGF5LmNhbGxiYWNrID0gaW5pdEdhbWU7XG5cblx0XHRvcHRpb25zLnNldHRpbmdzID0gbmV3IGlpby5SZWN0KHt4OiAyNTAsIHk6IHNwYWNpbmcqM30sIDE2MCwgNDUpO1xuXHRcdG9wdGlvbnMuc2V0dGluZ3MudGV4dCA9IG5ldyBpaW8uVGV4dCgnc2V0dGluZ3MnLCBvcHRpb25zLnNldHRpbmdzLnBvcyk7XG5cblx0XHRmb3IodmFyIGlpIGluIG9wdGlvbnMpIHtcblx0XHRcdHZhciBvcHRpb24gPSBvcHRpb25zW2lpXTtcblxuXHRcdFx0b3B0aW9uLnNldEZpbGxTdHlsZSgnIzVlM2Y2YicpO1xuXHRcdFx0aW8uYWRkVG9Hcm91cCgndWknLCBvcHRpb24sIDEwLCAxKTtcblxuXHRcdFx0b3B0aW9uLnRleHQuc2V0Rm9udCgnMzBweCBGcmVzY2EnKVxuXHRcdFx0XHQuc2V0RmlsbFN0eWxlKCd3aGl0ZScpXG5cdFx0XHRcdC5zZXRUZXh0QWxpZ24oJ2NlbnRlcicpXG5cdFx0XHRcdC50cmFuc2xhdGUoMCw4KTtcblx0XHRcdGlvLmFkZFRvR3JvdXAoJ3VpJywgb3B0aW9uLnRleHQsIDEwLCAxKTtcblx0XHR9XG5cblx0XHR2YXIgY2xpY2tIYW5kbGVyID0gZnVuY3Rpb24oZXZlbnQpIHtcblx0XHRcdHZhciBwb3MgPSBpby5nZXRFdmVudFBvc2l0aW9uKGV2ZW50LCAxKTtcblxuXHRcdFx0Zm9yKHZhciBpaSBpbiBvcHRpb25zKSB7XG5cdFx0XHRcdHZhciBvcHRpb24gPSBvcHRpb25zW2lpXTtcblxuXHRcdFx0XHRpZihvcHRpb24uY29udGFpbnMocG9zKSkge1xuXHRcdFx0XHRcdGNvbnNvbGUubG9nKG9wdGlvbi50ZXh0LnRleHQpO1xuXHRcdFx0XHRcdGlmKG9wdGlvbi5jYWxsYmFjaykge1xuXHRcdFx0XHRcdFx0aW8uY252c1sxXS5yZW1vdmVFdmVudExpc3RlbmVyKCdtb3VzZWRvd24nLCBjbGlja0hhbmRsZXIpO1xuXHRcdFx0XHRcdFx0b3B0aW9uLmNhbGxiYWNrKGlvKTtcblx0XHRcdFx0XHR9XG5cdFx0XHRcdH1cblx0XHRcdH1cblx0XHR9O1xuXHRcdGlvLmNudnNbMV0uYWRkRXZlbnRMaXN0ZW5lcignbW91c2Vkb3duJywgY2xpY2tIYW5kbGVyKTtcblx0fSxcblx0Z2FtZTogZnVuY3Rpb24gKGlvKSB7XG5cdFx0Y29uc29sZS5sb2coXCJVSSA+PiBnYW1lXCIpO1xuXG5cdFx0Ly9jbGVhciB0aGUgZ3JvdXAgb2YgYWxsIG9iamVjdHNcblx0XHRpby5jbGVhckdyb3VwKCd1aScsIDEpOyAvL25lZWRzIGlpbyBjb3JlIHVwZGF0ZSB0byB3b3JrXG5cdC8qXHR2YXIgdWkgPSBpby5nZXRHcm91cCgndWknLCAxKTsgLy93b3JrYXJvdW5kXG5cdFx0Zm9yKHZhciBpaT0wLCBsZW5ndGg9dWkubGVuZ3RoOyBpaTxsZW5ndGg7IGlpKyspIHtcblx0XHRcdHVpLnBvcCgpO1xuXHRcdH1cblx0Ki9cdGlvLmRyYXcoMSk7XG5cblx0XHR2YXIgd2lkdGggPSBpby5jYW52YXMud2lkdGg7XG5cdFx0dmFyIGhlaWdodCA9IGlvLmNhbnZhcy5oZWlnaHQ7XG5cblx0XHR2YXIgc2VjdGlvbiA9IG5ldyBpaW8uUmVjdCh3aWR0aC8yLCBoZWlnaHQqNy84LCB3aWR0aCwgaGVpZ2h0LzQpO1xuXHRcdHNlY3Rpb24uc2V0RmlsbFN0eWxlKCcjNWUzZjZiJyk7XG5cdFx0aW8uYWRkVG9Hcm91cCgndWknLCBzZWN0aW9uLCAxMCwgMSk7XG5cblx0XHR2YXIgbWFwID0gbmV3IGlpby5SZWN0KHdpZHRoKjEvOCwgaGVpZ2h0KjcvOCwgd2lkdGgvNC0zMCwgaGVpZ2h0LzQtMzApO1xuXHRcdG1hcC5zZXRGaWxsU3R5bGUoJ2JsYWNrJyk7XG5cdFx0aW8uYWRkVG9Hcm91cCgndWknLCBtYXAsIDEwLCAxKTtcblxuXHRcdHZhciBjb21tYW5kcyA9IG5ldyBpaW8uUmVjdCh3aWR0aCo3LzgsIGhlaWdodCo3LzgsIHdpZHRoLzQtMzAsIGhlaWdodC80LTMwKTtcblx0XHRjb21tYW5kcy5zZXRGaWxsU3R5bGUoJ2JsYWNrJyk7XG5cdFx0aW8uYWRkVG9Hcm91cCgndWknLCBjb21tYW5kcywgMTAsIDEpO1xuXHRcdGNvbW1hbmRzLmdyaWQgPSBuZXcgaWlvLkdyaWQod2lkdGgqMy80KzE1LCBoZWlnaHQqMy80KzE1LCA1LCAzLCBjb21tYW5kcy53aWR0aC81LCBjb21tYW5kcy5oZWlnaHQvMyk7XG5cdFx0Y29tbWFuZHMuZ3JpZC5zZXRTdHJva2VTdHlsZSgnIzVlM2Y2YicpO1xuXHRcdGlvLmFkZFRvR3JvdXAoJ3VpJywgY29tbWFuZHMuZ3JpZCwgMTAsIDEpO1xuXG5cdFx0dmFyIHVuaXQgPSBuZXcgaWlvLlJlY3Qod2lkdGgvMiwgaGVpZ2h0KjcvOCwgd2lkdGgvMiwgaGVpZ2h0LzQtMzApO1xuXHRcdHVuaXQuc2V0RmlsbFN0eWxlKCdibGFjaycpO1xuXHRcdGlvLmFkZFRvR3JvdXAoJ3VpJywgdW5pdCwgMTAsIDEpO1xuXG5cdFx0dmFyIGNyZWF0ZVJlc291cmNlVGV4dCA9IGZ1bmN0aW9uKHgsIHksIHcsIGgsIHRleHQpIHtcblx0XHRcdHZhciBvYmogPSBuZXcgaWlvLlJlY3QoeCwgeSwgdywgaClcblx0XHRcdFx0LnNldEZpbGxTdHlsZSgnIzVlM2Y2YicpO1xuXHRcdFx0b2JqLnRleHQgPSBuZXcgaWlvLlRleHQodGV4dClcblx0XHRcdFx0LnNldEZvbnQoJzE1cHggQ29uc29sYXMnKVxuXHRcdFx0XHQuc2V0VGV4dEFsaWduKCdjZW50ZXInKVxuXHRcdFx0XHQuc2V0RmlsbFN0eWxlKCd3aGl0ZScpO1xuXG5cdFx0XHRvYmouYWRkT2JqKG9iai50ZXh0LCBmYWxzZSwgaW8uY3R4c1sxXSwgMjcsIDUpO1xuXHRcdFx0cmV0dXJuIGlvLmFkZFRvR3JvdXAoJ3VpJywgb2JqLCAxMCwgMSk7XG5cdFx0fTtcblxuXHRcdHZhciBtaW5lcmFscyA9IGNyZWF0ZVJlc291cmNlVGV4dCh3aWR0aCo3LzgtNDAsIDIwLCAyMCwgMjAsICc1MDAnKTtcblx0XHR2YXIgZ2FzID0gY3JlYXRlUmVzb3VyY2VUZXh0KHdpZHRoKjcvOCszMCwgMjAsIDIwLCAyMCwgJzUwMCcpO1xuXHRcdHZhciBzdXBwbHkgPSBjcmVhdGVSZXNvdXJjZVRleHQod2lkdGgqNy84KzEwMCwgMjAsIDIwLCAyMCwgJzUvMTMnKTtcblx0fVxufTtcblxudmFyIGluaXRHYW1lID0gZnVuY3Rpb24gaW5pdEdhbWUoaW8pIHtcblx0SW50ZXJmYWNlLmdhbWUoaW8pO1xuXHRpby5ybXZBbGwoMCk7XG5cdGlvLmRyYXcoMCk7XG5cblx0dmFyIHQgPSBuZXcgaWlvLlRleHQoJ3BldyBwZXcnLCBpby5jYW52YXMuY2VudGVyKTtcblx0dC5zZXRGb250KCczMHB4IEZyZXNjYScpO1xuXHR0LnNldEZpbGxTdHlsZSgnd2hpdGUnKTtcblx0dC5zZXRUZXh0QWxpZ24oJ2NlbnRlcicpO1xuXHRpby5hZGRPYmoodCk7XG59O1xuXG52YXIgdWkgPSBmdW5jdGlvbiB1aShpbykge1xuXHRpby5hZGRDYW52YXMoMSk7IC8vIHpJbmRleCB0byBwdXQgVUkgb3ZlciBnYW1lIHdvcmxkXG5cdGlvLmFkZEdyb3VwKCd1aScsIDEwLCAxKTtcblxuXHRtb3VzZVJlY3QgPSBpby5hZGRUb0dyb3VwKCdtb3VzZScsIG5ldyBpaW8uUmVjdCgwLCAwLCAxMCksIDAsIDEpO1xuXG5cdGlvLmNudnNbMV0uYWRkRXZlbnRMaXN0ZW5lcignbW91c2Vtb3ZlJywgZnVuY3Rpb24oZXZlbnQpIHtcblx0XHRtb3VzZVJlY3Quc2V0UG9zKGlvLmdldEV2ZW50UG9zaXRpb24oZXZlbnQpKTtcblx0fSk7XG5cblx0Ly9pby5zZXRGcmFtZXJhdGUoNjAsIDEpO1xuXG5cdEludGVyZmFjZS5tZW51KGlvKTtcbn07XG5cbnZhciBlbmdpbmUgPSBmdW5jdGlvbihpbykge1xuXHRpby5hY3RpdmF0ZURlYnVnZ2VyKCk7XG5cblx0aW8uc2V0RnJhbWVyYXRlKDYwKTsgLy90ZXh0IGRvZXNuJ3QgcmVkcmF3P1xuXG5cdGlvLnNldEJHQ29sb3IoXCJncmVlblwiKTtcblx0dmFyIHQgPSBuZXcgaWlvLlRleHQoJ3RoYXQgZ3JlZW4gZ2FtZSBjYW52YXMnLCBpby5jYW52YXMuY2VudGVyKTtcblx0dC5zZXRGb250KCczMHB4IEZyZXNjYScpO1xuXHR0LnNldEZpbGxTdHlsZSgnd2hpdGUnKTtcblx0dC5zZXRUZXh0QWxpZ24oJ2NlbnRlcicpO1xuXHRpby5hZGRPYmoodCk7XG5cblx0dWkoaW8pO1xufTtcblxubW9kdWxlLmV4cG9ydHMgPSB7XG5cdHN0YXJ0OiBmdW5jdGlvbigpIHtcblx0XHRpaW8uc3RhcnQoZW5naW5lKTtcblx0fVxufTtcblxuIl19
;