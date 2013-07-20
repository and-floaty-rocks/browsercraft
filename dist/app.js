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


},{}]},{},[1])
//@ sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiZ2VuZXJhdGVkLmpzIiwic291cmNlcyI6WyIvVXNlcnMvZHN0cmVrL3d3dy9icm93c2VyY3JhZnQvbGliL2VudHJ5LmpzIiwiL1VzZXJzL2RzdHJlay93d3cvYnJvd3NlcmNyYWZ0L2xpYi9icm93c2VyY3JhZnQuanMiLCIvVXNlcnMvZHN0cmVrL3d3dy9icm93c2VyY3JhZnQvbGliL2VuZ2luZS5qcyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiO0FBQUE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ0xBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUN2QkE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBIiwic291cmNlc0NvbnRlbnQiOlsiKGZ1bmN0aW9uKCl7LypnbG9iYWwgd2luZG93OnRydWUqL1xud2luZG93LmJyb3dzZXJjcmFmdCA9IHJlcXVpcmUoJy4vYnJvd3NlcmNyYWZ0Jyk7XG53aW5kb3cuYnJvd3NlcmNyYWZ0LmdvX2dvKCk7XG5cblxufSkoKSIsIi8qXG4gKiBicm93c2VyY3JhZnRcbiAqIGFuZGZsb2F0eXJvY2tzLmNvbVxuICpcbiAqIENvcHlyaWdodCAoYykgMjAxMyBkc3RyZWtcbiAqIExpY2Vuc2VkIHVuZGVyIHRoZSBNSVQgbGljZW5zZS5cbiAqL1xuXG4oZnVuY3Rpb24oZXhwb3J0cykge1xuXHQndXNlIHN0cmljdCc7XG5cblx0dmFyIGVuZ2luZSA9IHJlcXVpcmUoJy4vZW5naW5lJyk7XG5cbiAgZXhwb3J0cy5hd2Vzb21lID0gZnVuY3Rpb24oKSB7XG4gICAgcmV0dXJuICdhd2Vzb21lJztcbiAgfTtcblxuXHRleHBvcnRzLmdvX2dvID0gZnVuY3Rpb24oKSB7XG5cdFx0ZW5naW5lLnN0YXJ0KCk7XG5cdH07XG5cbn0odHlwZW9mIGV4cG9ydHMgPT09ICdvYmplY3QnICYmIGV4cG9ydHMgfHwgdGhpcykpO1xuXG4iLCJ2YXIgZW5naW5lID0gZnVuY3Rpb24oaW8pIHtcblx0dmFyIHQgPSBuZXcgaWlvLlRleHQoJ2Jyb3dzZXJjcmFmdCcsIGlvLmNhbnZhcy5jZW50ZXIpO1xuXHR0LnNldEZvbnQoJzMwcHggQ29uc29sYXMnKTtcblx0dC5zZXRUZXh0QWxpZ24oJ2NlbnRlcicpO1xuXHR0LnNldEZpbGxTdHlsZSgnYmxhY2snKTtcblx0aW8uYWRkT2JqKHQpO1xufTtcblxubW9kdWxlLmV4cG9ydHMgPSB7XG5cdHN0YXJ0OiBmdW5jdGlvbigpIHtcblx0XHRpaW8uc3RhcnQoZW5naW5lKTtcblx0fVxufTtcblxuIl19
;