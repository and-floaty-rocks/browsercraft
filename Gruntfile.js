'use strict';

module.exports = function(grunt) {

  // Project configuration.
  grunt.initConfig({
    // Metadata.
    pkg: grunt.file.readJSON('package.json'),
    uglify: {
      vendor: {
        src: 'dist/vendor.js',
        dest: 'dist/vendor.min.js'
      },
      dist: {
        src: 'dist/<%= pkg.name %>.js',
        dest: 'dist/<%= pkg.name %>.min.js'
      }
    },
    nodeunit: {
      files: ['test/**/*_test.js']
    },
    jshint: {
      options: {
        jshintrc: '.jshintrc'
      },
      gruntfile: {
        src: 'Gruntfile.js'
      },
      lib: {
        options: {
          jshintrc: 'lib/.jshintrc'
        },
        src: ['lib/**/*.js']
      },
      test: {
        src: ['test/**/*.js']
      }
    },
    watch: {
      gruntfile: {
        files: '<%= jshint.gruntfile.src %>',
        tasks: ['jshint:gruntfile']
      },
      lib: {
        files: '<%= jshint.lib.src %>',
        tasks: ['jshint:lib', 'browserify', 'concat:dist']
      },
      test: {
        files: '<%= jshint.test.src %>',
        tasks: ['jshint:test', 'nodeunit']
      }
    },
		browserify: {
      client: {
        src: ['lib/entry.js'],
        dest: 'dist/app.js'
      }
    },
		concat: {
			vendor: {
        src: [
					'vendor/Box2dWeb-2.1.a.3.js', 
					'vendor/iioEngine.js', 
					'vendor/iioDebugger.js'
				],
				dest: 'dist/vendor.js'
			},
			dist: {
        src: [
					'dist/app.js'
				],
				dest: 'dist/<%= pkg.name %>.js'
			}
    }
  });

  // These plugins provide necessary tasks.
  grunt.loadNpmTasks('grunt-contrib-concat');
  grunt.loadNpmTasks('grunt-contrib-uglify');
  grunt.loadNpmTasks('grunt-contrib-nodeunit');
  grunt.loadNpmTasks('grunt-contrib-jshint');
  grunt.loadNpmTasks('grunt-contrib-watch');
	grunt.loadNpmTasks('grunt-browserify');

  // Default task.
  grunt.registerTask('default', ['jshint', 'nodeunit', 'browserify', 'concat:dist', 'uglify:dist']);
  grunt.registerTask('compile', ['jshint', 'nodeunit', 'browserify', 'concat', 'uglify']);

};
