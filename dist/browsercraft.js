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
;