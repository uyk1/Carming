package com.carming.backend.place.repository;

import com.carming.backend.course.dto.response.CoursePlaceResponse;
import com.carming.backend.place.domain.Place;
import com.carming.backend.place.dto.request.PlaceSearch;
import com.carming.backend.place.dto.response.popular.PopularPlaceDetailDto;
import com.carming.backend.place.dto.response.popular.PopularPlaceListDto;

import java.util.List;

public interface PlaceRepositoryCustom {

    List<Place> findPlaces(PlaceSearch search);

    List<Place> findPlacesByCourse(List<Long> placeKeys);

    List<PopularPlaceListDto> findPopular(Long size);

    PopularPlaceDetailDto findPopularPlaceDetail(Long id);

    List<String> findPlaceNames(List<Long> placeKeys);
}
