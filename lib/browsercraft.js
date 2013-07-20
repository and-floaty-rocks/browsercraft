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

