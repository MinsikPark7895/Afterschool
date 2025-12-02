package com.ssafy.afterschool;

import org.springframework.boot.SpringApplication;

public class TestAfterschoolApplication {

	public static void main(String[] args) {
		SpringApplication.from(AfterschoolApplication::main).with(TestcontainersConfiguration.class).run(args);
	}

}
