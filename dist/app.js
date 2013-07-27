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


},{}]},{},[1])
//@ sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiZ2VuZXJhdGVkLmpzIiwic291cmNlcyI6WyIvaG9tZS93ZWIvYnJvd3NlcmNyYWZ0L2xpYi9lbnRyeS5qcyIsIi9ob21lL3dlYi9icm93c2VyY3JhZnQvbGliL2Jyb3dzZXJjcmFmdC5qcyIsIi9ob21lL3dlYi9icm93c2VyY3JhZnQvbGliL2VuZ2luZS5qcyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiO0FBQUE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ0xBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUN2QkE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBIiwic291cmNlc0NvbnRlbnQiOlsiKGZ1bmN0aW9uKCl7LypnbG9iYWwgd2luZG93OnRydWUqL1xud2luZG93LmJyb3dzZXJjcmFmdCA9IHJlcXVpcmUoJy4vYnJvd3NlcmNyYWZ0Jyk7XG53aW5kb3cuYnJvd3NlcmNyYWZ0LmdvX2dvKCk7XG5cblxufSkoKSIsIi8qXG4gKiBicm93c2VyY3JhZnRcbiAqIGFuZGZsb2F0eXJvY2tzLmNvbVxuICpcbiAqIENvcHlyaWdodCAoYykgMjAxMyBkc3RyZWtcbiAqIExpY2Vuc2VkIHVuZGVyIHRoZSBNSVQgbGljZW5zZS5cbiAqL1xuXG4oZnVuY3Rpb24oZXhwb3J0cykge1xuXHQndXNlIHN0cmljdCc7XG5cblx0dmFyIGVuZ2luZSA9IHJlcXVpcmUoJy4vZW5naW5lJyk7XG5cbiAgZXhwb3J0cy5hd2Vzb21lID0gZnVuY3Rpb24oKSB7XG4gICAgcmV0dXJuICdhd2Vzb21lJztcbiAgfTtcblxuXHRleHBvcnRzLmdvX2dvID0gZnVuY3Rpb24oKSB7XG5cdFx0ZW5naW5lLnN0YXJ0KCk7XG5cdH07XG5cbn0odHlwZW9mIGV4cG9ydHMgPT09ICdvYmplY3QnICYmIGV4cG9ydHMgfHwgdGhpcykpO1xuXG4iLCJ2YXIgSW50ZXJmYWNlLCBpbml0R2FtZTtcbnZhciBtb3VzZVJlY3Q7XG5cbkludGVyZmFjZSA9IHtcblx0bWVudTogZnVuY3Rpb24oaW8pIHtcblx0XHRjb25zb2xlLmxvZygnVUkgPj4gbWVudScpO1xuXHRcdHZhciBvcHRpb25zID0ge307XG5cblx0XHRpby5ybXZGcm9tR3JvdXAoJ3VpJywgMSk7IC8vY2xlYXIgcHJldmlvdXMgVUlcblx0XHQvL2lvLmRyYXcoMSk7XG5cdFx0dmFyIHNwYWNpbmcgPSA1MDtcblxuXHRcdHZhciB0ID0gbmV3IGlpby5UZXh0KCdicm93c2VyY3JhZnQnLCBpby5jbnZzWzFdLndpZHRoLzIsIHNwYWNpbmcpO1xuXHRcdHQuc2V0Rm9udCgnMzBweCBGcmVzY2EnKTtcblx0XHR0LnNldEZpbGxTdHlsZSgnYmxhY2snKTtcblx0XHR0LnNldFRleHRBbGlnbignY2VudGVyJyk7XG5cdFx0aW8uYWRkVG9Hcm91cCgndWknLCB0LCAxMCwgMSk7XG5cblx0XHRvcHRpb25zLnBsYXkgPSBuZXcgaWlvLlJlY3Qoe3g6IDI1MCwgeTogc3BhY2luZyoyfSwgMTYwLCA0NSk7XG5cdFx0b3B0aW9ucy5wbGF5LnRleHQgPSBuZXcgaWlvLlRleHQoJ3BsYXknLCBvcHRpb25zLnBsYXkucG9zKTtcblx0XHRvcHRpb25zLnBsYXkuY2FsbGJhY2sgPSBpbml0R2FtZTtcblxuXHRcdG9wdGlvbnMuc2V0dGluZ3MgPSBuZXcgaWlvLlJlY3Qoe3g6IDI1MCwgeTogc3BhY2luZyozfSwgMTYwLCA0NSk7XG5cdFx0b3B0aW9ucy5zZXR0aW5ncy50ZXh0ID0gbmV3IGlpby5UZXh0KCdzZXR0aW5ncycsIG9wdGlvbnMuc2V0dGluZ3MucG9zKTtcblxuXHRcdGZvcih2YXIgaWkgaW4gb3B0aW9ucykge1xuXHRcdFx0dmFyIG9wdGlvbiA9IG9wdGlvbnNbaWldO1xuXG5cdFx0XHRvcHRpb24uc2V0RmlsbFN0eWxlKCcjNWUzZjZiJyk7XG5cdFx0XHRpby5hZGRUb0dyb3VwKCd1aScsIG9wdGlvbiwgMTAsIDEpO1xuXG5cdFx0XHRvcHRpb24udGV4dC5zZXRGb250KCczMHB4IEZyZXNjYScpXG5cdFx0XHRcdC5zZXRGaWxsU3R5bGUoJ3doaXRlJylcblx0XHRcdFx0LnNldFRleHRBbGlnbignY2VudGVyJylcblx0XHRcdFx0LnRyYW5zbGF0ZSgwLDgpO1xuXHRcdFx0aW8uYWRkVG9Hcm91cCgndWknLCBvcHRpb24udGV4dCwgMTAsIDEpO1xuXHRcdH1cblxuXHRcdHZhciBjbGlja0hhbmRsZXIgPSBmdW5jdGlvbihldmVudCkge1xuXHRcdFx0dmFyIHBvcyA9IGlvLmdldEV2ZW50UG9zaXRpb24oZXZlbnQsIDEpO1xuXG5cdFx0XHRmb3IodmFyIGlpIGluIG9wdGlvbnMpIHtcblx0XHRcdFx0dmFyIG9wdGlvbiA9IG9wdGlvbnNbaWldO1xuXG5cdFx0XHRcdGlmKG9wdGlvbi5jb250YWlucyhwb3MpKSB7XG5cdFx0XHRcdFx0Y29uc29sZS5sb2cob3B0aW9uLnRleHQudGV4dCk7XG5cdFx0XHRcdFx0aWYob3B0aW9uLmNhbGxiYWNrKSB7XG5cdFx0XHRcdFx0XHRpby5jbnZzWzFdLnJlbW92ZUV2ZW50TGlzdGVuZXIoJ21vdXNlZG93bicsIGNsaWNrSGFuZGxlcik7XG5cdFx0XHRcdFx0XHRvcHRpb24uY2FsbGJhY2soaW8pO1xuXHRcdFx0XHRcdH1cblx0XHRcdFx0fVxuXHRcdFx0fVxuXHRcdH07XG5cdFx0aW8uY252c1sxXS5hZGRFdmVudExpc3RlbmVyKCdtb3VzZWRvd24nLCBjbGlja0hhbmRsZXIpO1xuXHR9LFxuXHRnYW1lOiBmdW5jdGlvbiAoaW8pIHtcblx0XHRjb25zb2xlLmxvZyhcIlVJID4+IGdhbWVcIik7XG5cblx0XHQvL2NsZWFyIHRoZSBncm91cCBvZiBhbGwgb2JqZWN0c1xuXHRcdC8vaW8ucm12RnJvbUdyb3VwKCd1aScsIDEpOyAvL25lZWRzIGlpbyBjb3JlIHVwZGF0ZSB0byB3b3JrXG5cdFx0dmFyIHVpID0gaW8uZ2V0R3JvdXAoJ3VpJywgMSk7IC8vd29ya2Fyb3VuZFxuXHRcdGZvcih2YXIgaWk9MCwgbGVuZ3RoPXVpLmxlbmd0aDsgaWk8bGVuZ3RoOyBpaSsrKSB7XG5cdFx0XHR1aS5wb3AoKTtcblx0XHR9XG5cdFx0aW8uZHJhdygxKTtcblxuXHRcdHZhciB3aWR0aCA9IGlvLmNhbnZhcy53aWR0aDtcblx0XHR2YXIgaGVpZ2h0ID0gaW8uY2FudmFzLmhlaWdodDtcblxuXHRcdHZhciBzZWN0aW9uID0gbmV3IGlpby5SZWN0KHdpZHRoLzIsIGhlaWdodCo3LzgsIHdpZHRoLCBoZWlnaHQvNCk7XG5cdFx0c2VjdGlvbi5zZXRGaWxsU3R5bGUoJyM1ZTNmNmInKTtcblx0XHRpby5hZGRUb0dyb3VwKCd1aScsIHNlY3Rpb24sIDEwLCAxKTtcblxuXHRcdHZhciBtYXAgPSBuZXcgaWlvLlJlY3Qod2lkdGgqMS84LCBoZWlnaHQqNy84LCB3aWR0aC80LTMwLCBoZWlnaHQvNC0zMCk7XG5cdFx0bWFwLnNldEZpbGxTdHlsZSgnYmxhY2snKTtcblx0XHRpby5hZGRUb0dyb3VwKCd1aScsIG1hcCwgMTAsIDEpO1xuXG5cdFx0dmFyIGNvbW1hbmRzID0gbmV3IGlpby5SZWN0KHdpZHRoKjcvOCwgaGVpZ2h0KjcvOCwgd2lkdGgvNC0zMCwgaGVpZ2h0LzQtMzApO1xuXHRcdGNvbW1hbmRzLnNldEZpbGxTdHlsZSgnYmxhY2snKTtcblx0XHRpby5hZGRUb0dyb3VwKCd1aScsIGNvbW1hbmRzLCAxMCwgMSk7XG5cdFx0Y29tbWFuZHMuZ3JpZCA9IG5ldyBpaW8uR3JpZCh3aWR0aCozLzQrMTUsIGhlaWdodCozLzQrMTUsIDUsIDMsIGNvbW1hbmRzLndpZHRoLzUsIGNvbW1hbmRzLmhlaWdodC8zKTtcblx0XHRjb21tYW5kcy5ncmlkLnNldFN0cm9rZVN0eWxlKCcjNWUzZjZiJyk7XG5cdFx0aW8uYWRkVG9Hcm91cCgndWknLCBjb21tYW5kcy5ncmlkLCAxMCwgMSk7XG5cblx0XHR2YXIgdW5pdCA9IG5ldyBpaW8uUmVjdCh3aWR0aC8yLCBoZWlnaHQqNy84LCB3aWR0aC8yLCBoZWlnaHQvNC0zMCk7XG5cdFx0dW5pdC5zZXRGaWxsU3R5bGUoJ2JsYWNrJyk7XG5cdFx0aW8uYWRkVG9Hcm91cCgndWknLCB1bml0LCAxMCwgMSk7XG5cblx0XHR2YXIgY3JlYXRlUmVzb3VyY2VUZXh0ID0gZnVuY3Rpb24oeCwgeSwgdywgaCwgdGV4dCkge1xuXHRcdFx0dmFyIG9iaiA9IG5ldyBpaW8uUmVjdCh4LCB5LCB3LCBoKVxuXHRcdFx0XHQuc2V0RmlsbFN0eWxlKCcjNWUzZjZiJyk7XG5cdFx0XHRvYmoudGV4dCA9IG5ldyBpaW8uVGV4dCh0ZXh0KVxuXHRcdFx0XHQuc2V0Rm9udCgnMTVweCBDb25zb2xhcycpXG5cdFx0XHRcdC5zZXRUZXh0QWxpZ24oJ2NlbnRlcicpXG5cdFx0XHRcdC5zZXRGaWxsU3R5bGUoJ3doaXRlJyk7XG5cblx0XHRcdG9iai5hZGRPYmoob2JqLnRleHQsIGZhbHNlLCBpby5jdHhzWzFdLCAyNywgNSk7XG5cdFx0XHRyZXR1cm4gaW8uYWRkVG9Hcm91cCgndWknLCBvYmosIDEwLCAxKTtcblx0XHR9O1xuXG5cdFx0dmFyIG1pbmVyYWxzID0gY3JlYXRlUmVzb3VyY2VUZXh0KHdpZHRoKjcvOC00MCwgMjAsIDIwLCAyMCwgJzUwMCcpO1xuXHRcdHZhciBnYXMgPSBjcmVhdGVSZXNvdXJjZVRleHQod2lkdGgqNy84KzMwLCAyMCwgMjAsIDIwLCAnNTAwJyk7XG5cdFx0dmFyIHN1cHBseSA9IGNyZWF0ZVJlc291cmNlVGV4dCh3aWR0aCo3LzgrMTAwLCAyMCwgMjAsIDIwLCAnNS8xMycpO1xuXHR9XG59O1xuXG52YXIgaW5pdEdhbWUgPSBmdW5jdGlvbiBpbml0R2FtZShpbykge1xuXHRJbnRlcmZhY2UuZ2FtZShpbyk7XG5cdGlvLnJtdkFsbCgwKTtcblx0aW8uZHJhdygwKTtcblxuXHR2YXIgdCA9IG5ldyBpaW8uVGV4dCgncGV3IHBldycsIGlvLmNhbnZhcy5jZW50ZXIpO1xuXHR0LnNldEZvbnQoJzMwcHggRnJlc2NhJyk7XG5cdHQuc2V0RmlsbFN0eWxlKCd3aGl0ZScpO1xuXHR0LnNldFRleHRBbGlnbignY2VudGVyJyk7XG5cdGlvLmFkZE9iaih0KTtcbn07XG5cbnZhciB1aSA9IGZ1bmN0aW9uIHVpKGlvKSB7XG5cdGlvLmFkZENhbnZhcygxKTsgLy8gekluZGV4IHRvIHB1dCBVSSBvdmVyIGdhbWUgd29ybGRcblx0aW8uYWRkR3JvdXAoJ3VpJywgMTAsIDEpO1xuXG5cdG1vdXNlUmVjdCA9IGlvLmFkZFRvR3JvdXAoJ21vdXNlJywgbmV3IGlpby5SZWN0KDAsIDAsIDEwKSwgMCwgMSk7XG5cblx0aW8uY252c1sxXS5hZGRFdmVudExpc3RlbmVyKCdtb3VzZW1vdmUnLCBmdW5jdGlvbihldmVudCkge1xuXHRcdG1vdXNlUmVjdC5zZXRQb3MoaW8uZ2V0RXZlbnRQb3NpdGlvbihldmVudCkpO1xuXHRcdC8vY29uc29sZS5sb2cobW91c2VSZWN0KTtcblx0fSk7XG5cblx0Ly9pby5zZXRGcmFtZXJhdGUoNjAsIDEpO1xuXG5cdEludGVyZmFjZS5tZW51KGlvKTtcbn07XG5cbnZhciBlbmdpbmUgPSBmdW5jdGlvbihpbykge1xuXHRpby5hY3RpdmF0ZURlYnVnZ2VyKCk7XG5cblx0aW8uc2V0RnJhbWVyYXRlKDYwKTsgLy90ZXh0IGRvZXNuJ3QgcmVkcmF3P1xuXG5cdGlvLnNldEJHQ29sb3IoXCJncmVlblwiKTtcblx0dmFyIHQgPSBuZXcgaWlvLlRleHQoJ3RoYXQgZ3JlZW4gZ2FtZSBjYW52YXMnLCBpby5jYW52YXMuY2VudGVyKTtcblx0dC5zZXRGb250KCczMHB4IEZyZXNjYScpO1xuXHR0LnNldEZpbGxTdHlsZSgnd2hpdGUnKTtcblx0dC5zZXRUZXh0QWxpZ24oJ2NlbnRlcicpO1xuXHRpby5hZGRPYmoodCk7XG5cblx0dWkoaW8pO1xufTtcblxubW9kdWxlLmV4cG9ydHMgPSB7XG5cdHN0YXJ0OiBmdW5jdGlvbigpIHtcblx0XHRpaW8uc3RhcnQoZW5naW5lKTtcblx0fVxufTtcblxuIl19
;