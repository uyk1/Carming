import {View, Text, Button} from 'react-native';
import {useDispatch, useSelector} from 'react-redux';
import {logout} from '../redux/slices/authSlice';
import {RootState} from '../redux/store';

const HomeScreen = () => {
  const dispatch = useDispatch();
  const {member, token} = useSelector((state: RootState) => state.auth);

  const handleLogout = () => {
    dispatch(logout());
  };

  return (
    <View>
      <Text>{member?.name}님 환영합니다.</Text>
      <Text>로그인 토큰: {token}</Text>
      <Button title="로그아웃" onPress={handleLogout} />
    </View>
  );
};

export default HomeScreen;
