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

	io.setFramerate(60);

	io.activateDebugger();
};

module.exports = {
	start: function() {
		iio.start(engine);
	}
};


},{}]},{},[1])
//@ sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiZ2VuZXJhdGVkLmpzIiwic291cmNlcyI6WyIvVXNlcnMvZHN0cmVrL3d3dy9icm93c2VyY3JhZnQvbGliL2VudHJ5LmpzIiwiL1VzZXJzL2RzdHJlay93d3cvYnJvd3NlcmNyYWZ0L2xpYi9icm93c2VyY3JhZnQuanMiLCIvVXNlcnMvZHN0cmVrL3d3dy9icm93c2VyY3JhZnQvbGliL2VuZ2luZS5qcyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiO0FBQUE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ0xBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUN2QkE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0EiLCJzb3VyY2VzQ29udGVudCI6WyIoZnVuY3Rpb24oKXsvKmdsb2JhbCB3aW5kb3c6dHJ1ZSovXG53aW5kb3cuYnJvd3NlcmNyYWZ0ID0gcmVxdWlyZSgnLi9icm93c2VyY3JhZnQnKTtcbndpbmRvdy5icm93c2VyY3JhZnQuZ29fZ28oKTtcblxuXG59KSgpIiwiLypcbiAqIGJyb3dzZXJjcmFmdFxuICogYW5kZmxvYXR5cm9ja3MuY29tXG4gKlxuICogQ29weXJpZ2h0IChjKSAyMDEzIGRzdHJla1xuICogTGljZW5zZWQgdW5kZXIgdGhlIE1JVCBsaWNlbnNlLlxuICovXG5cbihmdW5jdGlvbihleHBvcnRzKSB7XG5cdCd1c2Ugc3RyaWN0JztcblxuXHR2YXIgZW5naW5lID0gcmVxdWlyZSgnLi9lbmdpbmUnKTtcblxuICBleHBvcnRzLmF3ZXNvbWUgPSBmdW5jdGlvbigpIHtcbiAgICByZXR1cm4gJ2F3ZXNvbWUnO1xuICB9O1xuXG5cdGV4cG9ydHMuZ29fZ28gPSBmdW5jdGlvbigpIHtcblx0XHRlbmdpbmUuc3RhcnQoKTtcblx0fTtcblxufSh0eXBlb2YgZXhwb3J0cyA9PT0gJ29iamVjdCcgJiYgZXhwb3J0cyB8fCB0aGlzKSk7XG5cbiIsInZhciBlbmdpbmUgPSBmdW5jdGlvbihpbykge1xuXHR2YXIgdCA9IG5ldyBpaW8uVGV4dCgnYnJvd3NlcmNyYWZ0JywgaW8uY2FudmFzLmNlbnRlcik7XG5cdHQuc2V0Rm9udCgnMzBweCBDb25zb2xhcycpO1xuXHR0LnNldFRleHRBbGlnbignY2VudGVyJyk7XG5cdHQuc2V0RmlsbFN0eWxlKCdibGFjaycpO1xuXHRpby5hZGRPYmoodCk7XG5cblx0aW8uc2V0RnJhbWVyYXRlKDYwKTtcblxuXHRpby5hY3RpdmF0ZURlYnVnZ2VyKCk7XG59O1xuXG5tb2R1bGUuZXhwb3J0cyA9IHtcblx0c3RhcnQ6IGZ1bmN0aW9uKCkge1xuXHRcdGlpby5zdGFydChlbmdpbmUpO1xuXHR9XG59O1xuXG4iXX0=
;