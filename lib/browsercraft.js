/*
 * browsercraft
 * andfloatyrocks.com
 *
 * Copyright (c) 2013 dstrek
 * Licensed under the MIT license.
 */

(function(exports) {

  'use strict';

	var uuid = require('uuid');

  exports.awesome = function() {
    return 'awesome';
  };

	exports.uuid = function() {
		return uuid.v4();
	};

	exports.go_go = function() {
		console.log(uuid.v4());
	};

}(typeof exports === 'object' && exports || this));
